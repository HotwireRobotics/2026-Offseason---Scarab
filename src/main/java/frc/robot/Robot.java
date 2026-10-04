package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightHelpers;
import frc.robot.hotwire.Logs;
import frc.robot.applicable.ctre.Drive.Side;
import frc.robot.applicable.ctre.Drive.Zone;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  // Declare autonomous command.
  private Command autonomousCommand;

  // Declare robot container.
  private final RobotContainer container;

  // Initialize field object.
  private final Field2d field = new Field2d();

  public Robot() {
    // Record metadata.
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers and replay source.
    switch (Constants.mode) {
      case REAL:
        // Proper case logging.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Simulate logging.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;

      default:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
    }

    // Start logging.
    Logger.start();

    // Initialize robot container.
    container = new RobotContainer();

    // Log pose on field.
    SmartDashboard.putData("Robot Pose (Field)", field);
  }

  @Override
  public void robotPeriodic() {
    // Track time.
    Time time = Seconds.of(Timer.getTimestamp());

    // Control command scheduler and log data.
    CommandScheduler.getInstance().run();

    // Dispatch any newly transcribed voice command.
    container.voice.poll();

    // Log pose data.
    Logger.recordOutput("Robot Pose", container.drive.getPose());
    Logger.recordOutput("IsNeutral", container.drive.getZone().equals(Zone.NEUTRAL));
    Logger.recordOutput("IsRightSide", container.drive.getSide().equals(Side.RIGHT));

    // Log field poses.
    Logger.recordOutput("Hub Pose", Constants.Poses.hub.getPose());
    Logger.recordOutput("Tower Pose", Constants.Poses.tower.getPose());

    // Update python pose estimate.
    Double[] robotpose = {
        container.drive.getPose().getX(),
        container.drive.getPose().getX(),
        container.drive.getRotation().getDegrees()
    };
    SmartDashboard.putNumberArray("robot-pose", robotpose);

    // Update field visualization.
    field.setRobotPose(container.drive.getPose());
    // container.simulation.tick();
  }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");

    // Drop any voice command so it cannot outlive the enable that heard it.
    container.voice.clear();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    Logger.recordOutput("Robot/Mode", "Autonomous");

    // Get set autonomous command.
    autonomousCommand = container.getAutonomousCommand();

    // Seed starting pose for pathfollowing.
    container.seedAutonomousPose(autonomousCommand);

    // Schedule autonomous command.
    if (autonomousCommand != null) {
      Logger.recordOutput("Robot/AutonomousCommand", autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      Logger.recordOutput("Robot/AutonomousCommand", "None");
    }

    // container.simulation.autonomous();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    Logger.recordOutput("Robot/Mode", "Teleop");

    // Halt autonomous command.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    Logger.recordOutput("Robot/Mode", "Test");

    // Cancel all existing commands.
    CommandScheduler.getInstance().cancelAll();

    // Point to teleoperated.
    teleopInit();
  }

  @Override
  public void testPeriodic() {
    // Point to teleoperated.
    teleopPeriodic();
  }

  @Override
  public void simulationInit() {
    // Enable the simulated robot so mechanisms and voice commands can be
    // exercised without a Driver Station attached. WPILib only calls this
    // method under simulation, so the real robot never reaches it.
    //
    // Skipped when a Driver Station is already attached, and overwritten by the
    // next packet from one that attaches later, so the Driver Station keeps
    // authority over enable and disable.
    if (!DriverStation.isDSAttached()) {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
