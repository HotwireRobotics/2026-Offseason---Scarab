package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.applicable.ctre.DriveCommands;
import frc.robot.applicable.simulation.Handler;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Joysticks;
import frc.robot.constants.Constants.Mode;
import frc.robot.hotwire.Voice;
import frc.robot.applicable.ctre.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.actuator.Actuator;
import frc.robot.constants.Constants.Joysticks.*;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    // Declare subsystems.
    public final Drive drive;
    public final Vision vision;
    public final Intake intake;
    public final Hopper hopper;
    public final Shooter shooter;
    public final Actuator actuator;
    public final PowerDistribution PDP;

    // Voice interface, and the flag its test verb toggles.
    public final Voice voice = new Voice();
    private boolean test = false;

    // Dashboard inputs.
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Boolean> localization;
    private final LoggedDashboardChooser<Boolean> alignment;
    /** Held to shoot. */
    private final Trigger shooting = mirrored(Joysticks.operator.rightTrigger(), Joysticks.driver.rightTrigger())
            .or(Joysticks.operator.rightBumper());
    /** Held to aim at the hub, and range the shooter off the hub distance. */
    private final Trigger aiming = Joysticks.operator.x().or(Joysticks.driver.x());

    /**
     * An operator control, also on the driver controller while
     * {@link Constants#testing} is set.
     */
    private static Trigger mirrored(Trigger operator, Trigger driver) {
        return Constants.testing ? operator.or(driver) : operator;
    }

    public RobotContainer() {
        // Initialize subsystems.
        drive = new Drive(Constants.mode);
        shooter = new Shooter(
                shooting,
                // Held, the button that aims at the hub also takes the
                // shooter's velocity off the range it is aiming from.
                aiming,
                this::getHubDistance);
        vision = new Vision(
                drive::getPose, drive::getRotation,
                drive::addVisionMeasurement);
        intake = new Intake(
                mirrored(Joysticks.operator.leftTrigger(), Joysticks.driver.leftTrigger()));
        hopper = new Hopper(
                mirrored(Joysticks.operator.leftBumper(), Joysticks.driver.leftBumper())
                //   .or(new Trigger(() -> shooter.isReady())) //! Disabled auto-start.
        );
        actuator = new Actuator(Joysticks.operator.a());
        PDP = new PowerDistribution();
        PDP.setSwitchableChannel(true);

        // Configure button bindings.
        configureButtonBindings();
        configureVoiceBindings();

        // Configure dashboard inputs.
        alignment = new LoggedDashboardChooser<>("Dashboard/alignment", new SendableChooser<Boolean>());
        alignment.addDefaultOption("Required", true);
        alignment.addOption("Supersede", false);
        // TODO: Add on-change method for alignment requirement.

        localization = new LoggedDashboardChooser<>("Dashboard/localization", new SendableChooser<Boolean>());
        localization.addDefaultOption("Enabled", true);
        localization.addOption("Disabled", false);
        localization.onChange(v -> vision.setEnabled(v));
        // Declare drivetrain pathplanner events.
        final Command stopDrive = Commands.runOnce(() -> drive.stop());
        final Command lockDrive = Commands.runOnce(() -> drive.stopWithX());

        final Command intaking = intake.run().onlyWhile(DriverStation::isAutonomousEnabled);
        NamedCommands.registerCommand("Start Intaking",
                Commands.runOnce(() -> CommandScheduler.getInstance().schedule(intaking)));
        NamedCommands.registerCommand("Stop Intaking",
                Commands.runOnce(intaking::cancel).andThen(intake.stop()));

        // Actuator (intake deployment) auto markers.
        NamedCommands.registerCommand("Lower Intake", actuator.runExtend());
        NamedCommands.registerCommand("Drop Arm",     actuator.runExtend());

        NamedCommands.registerCommand("Intake Period",   Commands.none());
        NamedCommands.registerCommand("Occilate Intake", Commands.none());

        // Shooter auto markers (non-blocking, same reasoning as the intake).
        final Command shooting = shooter.run().onlyWhile(DriverStation::isAutonomousEnabled);
        NamedCommands.registerCommand("Start Shooting",
                Commands.runOnce(() -> CommandScheduler.getInstance().schedule(shooting)));
        NamedCommands.registerCommand("Stop Shooting",
                Commands.runOnce(shooting::cancel).andThen(shooter.stop()));

        NamedCommands.registerCommand("Firing Sequence", Commands.sequence(
			Commands.parallel(
				drive.stopX(),
				shooter.runAt(Constants.Shooter.kSpeed),
				Commands.sequence(
					waitFor(Constants.Shooter.kSpinUpTime),
					Commands.parallel(
						hopper.run(),
						Commands.sequence(
							waitFor(Constants.Shooter.kRetractDelay), 
							Commands.runOnce(actuator::retract)))))
				.raceWith(Commands.sequence(
					waitFor(Constants.Shooter.kSpinUpTime),
					waitFor(Constants.Shooter.kFiringTime))),
			Commands.parallel(
					shooter.stop(),
					hopper .stop()).withTimeout(0.1)));

        // Autonomous
        if (!Constants.mode.equals(Mode.COMPETITION)) {

            // Create autonomous selector and add options.
            autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>()); // new
            // SendableChooser<Command>()

            // Drivetrain characterization routines.
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
            autoChooser.addOption(
                    "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        } else {
            autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        }

        // Primary autonomous routine.
        autoChooser.addOption("A-Unineutral Right", new PathPlannerAuto("A-Unineutral", false));
        autoChooser.addOption("A-Unineutral Left", new PathPlannerAuto("A-Unineutral", true));

        // Secondary routine.
        autoChooser.addOption("A-Back-Up", new PathPlannerAuto("A-Back-Up", false));
    }

    /** Distance from the robot to the hub. */
    private Distance getHubDistance() {
        return Meters.of(drive.getPose().getTranslation()
                .getDistance(Constants.Poses.hub.getPose().getTranslation()));
    }

    /**
     * Rotation the chassis is turned by, on top of the bearing to the hub, to
     * put the shooter on target. k180deg aims the back of the chassis, which is
     * what Hotwire does; set it to kZero if this robot shoots forward.
     */
    private static final Rotation2d kShooterOffset = Rotation2d.kZero;

    /** Returns the Rotation2d the robot needs to face the hub. */
    private Rotation2d calculateHubRotation() {
        // Get poses.
        Pose2d robotPose = drive.getPose();
        Pose2d hubPose = Constants.Poses.hub.getPose();

        // Pose differences.
        double dx = hubPose.getX() - robotPose.getX();
        double dy = hubPose.getY() - robotPose.getY();

        // Bearing from the robot to the hub, then the shooter's own offset.
        Rotation2d bearing = new Rotation2d(
                Radians.of(Math.IEEEremainder(
                        Math.atan2(dy, dx),
                        Constants.Mathematics.TAU)));
        Rotation2d rotation = bearing.rotateBy(kShooterOffset);

        // Log the pointer.
        Pose2d pointer = new Pose2d(robotPose.getX(), robotPose.getY(), rotation);
        Logger.recordOutput("Hub Pointer", pointer);

        Logger.recordOutput("Align/Alliance", Constants.getAlliance());
        Logger.recordOutput("Align/Hub Pose", hubPose);
        Logger.recordOutput("Align/Bearing", bearing.getDegrees());
        Logger.recordOutput("Align/Target", rotation.getDegrees());
        Logger.recordOutput("Align/Measured", drive.getRotation().getDegrees());
        Logger.recordOutput("Align/Error", rotation.minus(drive.getRotation()).getDegrees());

        // Update drive target.
        drive.setRotationTarget(rotation);

        return drive.getRotationTarget();
    }

    /**
     * Orient the robot to face a supplied angle.
     *
     * @param rotation
     */
    private Command pointToAngle(Supplier<Rotation2d> rotation) {
        return DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -Constants.Joysticks.driver.getLeftY(),
                () -> -Constants.Joysticks.driver.getLeftX(),
                rotation)
                .beforeStarting(() -> drive.setHeadingSource(Drive.HeadingSource.GYRO)) //! Disabled kinematic compensation.
                .finallyDo(() -> drive.setHeadingSource(Drive.HeadingSource.GYRO));
    }

    /** Orient robot to face the hub. */
    private Command firingOrientation() {
        return pointToAngle(this::calculateHubRotation);
    }

    private void configureButtonBindings() {
        // Third person drive command.
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -Constants.Joysticks.driver.getLeftY(),
                        () -> -Constants.Joysticks.driver.getLeftX(),
                        () -> -Constants.Joysticks.driver.getRightX()));

        // Aim at the hub while held.
        aiming.whileTrue(firingOrientation());

        Constants.Joysticks.driver
                .a()
                .onTrue(Commands.runOnce(drive::rezero, drive)
                        .ignoringDisable(true));
    }

    /**
     * Bind spoken verbs to subsystem commands. Durations are spoken; the
     * fallbacks below apply to a phrase that carries none.
     */
    private void configureVoiceBindings() {
        // Mechanisms, run for the spoken duration.
        voice.bind("intake",  time -> intake.run().withTimeout(time.orElse(Seconds.of(5))));
        voice.bind("hopper",  time -> hopper.run().withTimeout(time.orElse(Seconds.of(5))));
        voice.bind("shooter", time -> shooter.run()
                .withTimeout(time.orElse(Constants.Shooter.kFiringTime.get())));

        // Actuator states. Both are momentary; Actuator.periodic() holds the
        // commanded position from there on.
        voice.bind("extend",  time -> Commands.runOnce(actuator::extend));
        //! Disabled
        // voice.bind("retract", time -> Commands.runOnce(actuator::retract));

        // Flips a flag on NetworkTables and moves nothing.
        voice.bind("test", time -> Commands.runOnce(() -> {
                test = !test;
                Logger.recordOutput("Voice/Test", test);
            }).ignoringDisable(true));

        // Halt every mechanism and the drivetrain.
        voice.bind("stop", time -> Commands.parallel(
                intake .stop(),
                hopper .stop(),
                shooter.stop(),
                Commands.runOnce(() -> drive.stop())).ignoringDisable(true));
    }

    /**
     * A wait whose length is read when it runs rather than when it is built.
     *
     * <p>Auto markers are registered once at startup, so a duration read there
     * is the one the robot booted with. Reading it at the marker is what lets a
     * value tuned between runs reach the next one.
     *
     * @param duration Tunable the wait reads.
     */
    private static Command waitFor(Supplier<Time> duration) {
        return Commands.defer(
                () -> Commands.waitSeconds(duration.get().in(Seconds)), Set.of());
    }

    /**
     * Supplies the autonomous command selected on the dashboard.
     *
     * @return
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Set the robot's pose to the starting pose of the selected autonomous
     * command, if it exists.
     *
     * @param autonomousCommand
     */
    public void seedAutonomousPose(Command autonomousCommand) {
        if (!(autonomousCommand instanceof PathPlannerAuto selectedAuto)) {
            return;
        }

        // Get autonomous starting pose.
        Pose2d startingPose = selectedAuto.getStartingPose();
        if (startingPose == null) {
            return;
        }

        drive.setPose(startingPose);
        Logger.recordOutput("AutoSeedPose", startingPose);
    }
}

// ./gradlew deploy --no-daemon
// ./gradlew simulateExternalJavaRelease
