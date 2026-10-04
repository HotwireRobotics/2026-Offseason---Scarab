package frc.robot.applicable.ctre;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.applicable.ctre.generated.TunerConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Field;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.LimelightHelpers.PoseEstimate;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 57.334076;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  public final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  // Heading odometry integrates, from whichever source is selected below. It is
  // not the gyro's heading: a stretch run off the modules leaves the two apart,
  // which is the point of keeping them separate.
  private Rotation2d odometryRotation = Rotation2d.kZero;

  // Offset from the gyro's own yaw to the heading it reports. Only rezero()
  // moves it, so neither a pose reset nor odometry integrating the modules can
  // quietly re-aim the gyro.
  private Rotation2d gyroOffset = Rotation2d.kZero;

  /**
   * Direction the modules take rotation in: +1 when the turn they are given,
   * and the turn they report, match the one the chassis makes, and -1 when this
   * drivetrain does the opposite of both.
   *
   * <p>It reaches every place the modules speak for the chassis's rotation --
   * what the kinematics are handed, and what they read back out for odometry
   * and for the measured speeds a path follower watches. Those move together:
   * flipping the measurement alone leaves the heading controller watching its
   * own command move away from the target, which runs away instead of settling.
   *
   * <p>The gyro is deliberately outside it. The gyro is a separate instrument
   * with its own convention, and it is already right: taking it the other way
   * negates the heading, which leaves the field-relative conversion turning a
   * request meant for straight ahead into one at twice the heading -- fine at
   * zero, sideways at 45 degrees, straight backwards at 90.
   */
  private static final double kModuleRotationSign = -1.0;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, odometryRotation, lastModulePositions, Pose2d.kZero);

  private Rotation2d rotationTarget = Rotation2d.kZero;

  // Two independent heading integrals, unwrapped so they stay comparable
  // across multiple full rotations. The module-derived one carries no gyro
  // scale error, so the divergence between them over a known number of
  // rotations measures that error directly, in degrees per rotation.
  private double kinematicYawDegrees = 0.0;
  private double measuredYawDegrees = 0.0;
  private Rotation2d lastGyroSample = Rotation2d.kZero;
  private boolean gyroSeeded = false;

  /**
   * What the pose estimate's heading follows. The two readings are kept apart,
   * and the gyro is never on the receiving end: it hands the estimate a heading
   * and takes nothing back.
   */
  public enum HeadingSource {
    /** The gyro's heading, taken outright each sample. */
    GYRO,
    /**
     * Yaw extrapolated from module deltas, carrying on from the gyro heading
     * the estimate last held. This is what the drivetrain runs on when no gyro
     * is connected: no gyro scale or mount error, but it drifts with scrub, so
     * it is only trusted for the length of an alignment.
     */
    KINEMATIC
  }

  private HeadingSource headingSource = HeadingSource.GYRO;

  /**
   * Selects what the estimate's heading follows.
   *
   * <p>Leaving the gyro seeds the modules from the heading the gyro last gave,
   * and the estimate extrapolates from there. Returning to it hands the heading
   * back to the gyro, stepping the estimate by whatever the modules drifted in
   * between; the gyro itself is untouched either way.
   */
  public void setHeadingSource(HeadingSource source) {
    headingSource = source;
  }

  public HeadingSource getHeadingSource() {
    return headingSource;
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            Constants.Control.translationPID, Constants.Control.rotationPID),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public Drive(GyroIO gyro, ModuleIO[] modules) {
    this(
      gyro,
      modules[0], modules[1],
      modules[2], modules[3]
    );
  }

  public Drive(Constants.Mode mode) {
    this(switch (mode) {
      case REAL -> new GyroIOPigeon2();
      case SIM -> new GyroIO() {};
      default -> new GyroIOPigeon2();
    },
    switch (mode) {
      case REAL -> new ModuleIO[] {
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight)
      };
      case SIM -> new ModuleIO[] {
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight)
      };
      default -> new ModuleIO[] {
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight)
      };
    });
    // this(
    //     new GyroIOPigeon2(),
    //     new ModuleIOTalonFX(TunerConstants.FrontLeft),
    //     new ModuleIOTalonFX(TunerConstants.FrontRight),
    //     new ModuleIOTalonFX(TunerConstants.BackLeft),
    //     new ModuleIOTalonFX(TunerConstants.BackRight));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Both sources are tracked every sample, whichever one odometry uses, so
      // that switching between them never folds a whole interval into one step.
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      Rotation2d kinematicDelta = new Rotation2d(kModuleRotationSign * twist.dtheta);
      kinematicYawDegrees += kModuleRotationSign * Math.toDegrees(twist.dtheta);

      // The unwrapped integral below stays on the raw device, so a rezero mid
      // match shifts the heading without stepping the scale measurement.
      Rotation2d gyroHeading = null;
      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        Rotation2d sample = gyroInputs.odometryYawPositions[i];
        if (gyroSeeded) {
          measuredYawDegrees += sample.minus(lastGyroSample).getDegrees();
        }
        lastGyroSample = sample;
        gyroSeeded = true;
        gyroHeading = sample.plus(gyroOffset);
      }

      // Take the gyro's heading outright while it is selected and reporting, so
      // the estimate holds no heading of its own to drift. Otherwise extrapolate
      // from the modules, carrying on from the last heading the gyro gave -- the
      // seed an alignment starts from. Nothing travels back the other way.
      odometryRotation = (headingSource == HeadingSource.GYRO && gyroHeading != null)
          ? gyroHeading
          : odometryRotation.plus(kinematicDelta);

      // Flip for red alliance to match vision coordinate system
      // if (!Constants.currentMode.equals(Constants.Mode.SIM)
      //     && DriverStation.getAlliance().isPresent()
      //     && DriverStation.getAlliance().get() == Alliance.Red) {
      //   odometryRotation = odometryRotation.plus(Rotation2d.kPi);
      // }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], odometryRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.mode != Mode.SIM);

    Logger.recordOutput("Gyro", getGyroRotation());
    Logger.recordOutput("Gyro/Offset", gyroOffset);
    Logger.recordOutput("Odometry/Heading", odometryRotation);
    Logger.recordOutput("Odometry/Heading Error",
        odometryRotation.minus(getGyroRotation()).getDegrees());
    Logger.recordOutput("Gyro/Kinematic Yaw", kinematicYawDegrees);
    Logger.recordOutput("Gyro/Measured Yaw", measuredYawDegrees);
    Logger.recordOutput("Gyro/Yaw Error", measuredYawDegrees - kinematicYawDegrees);
    Logger.recordOutput("Gyro/Heading Source", headingSource);
  }

  /**
   * The gyro's heading: its own yaw, shifted by the offset the driver sets.
   * Read straight off the device, so it stays the gyro's answer even while
   * odometry is integrating the modules instead.
   */
  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition.plus(gyroOffset);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Turn the request into the drivetrain's own direction of rotation before
    // anything is computed from it, so the modules and the heading read back
    // off them are describing the same turn.
    ChassisSpeeds commanded =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            kModuleRotationSign * speeds.omegaRadiansPerSecond);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(commanded, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints, and the request as asked for rather than as
    // handed to the modules, so it lines up with the measured speeds above.
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command stopX() {
    return run(() -> stopWithX());
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Returns the measured chassis speeds of the robot, in the same direction of
   * rotation the commands are given in. A path follower closes its own loop on
   * these, so they have to come back turning the way it asked.
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds measured = kinematics.toChassisSpeeds(getModuleStates());
    return new ChassisSpeeds(
        measured.vxMetersPerSecond,
        measured.vyMetersPerSecond,
        kModuleRotationSign * measured.omegaRadiansPerSecond);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public void setRotationTarget(Rotation2d target) {
    rotationTarget = target;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Zero the heading, the gyro's and the pose estimate's together. This is the
   * only place the gyro offset moves; a pose reset from auto, a path or vision
   * re-aims the estimate alone and leaves the gyro reading what it reads.
   */
  public void rezero() {
    gyroOffset = gyroInputs.yawPosition.unaryMinus();

    // Carry the estimate to the gyro's new heading before rebasing the pose on
    // it, so the estimator is not handed a heading the next sample contradicts.
    odometryRotation = getGyroRotation();
    setPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    Logger.recordOutput("Odometry/PoseReset", pose);
    poseEstimator.resetPosition(odometryRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(PoseEstimate estimate, Matrix<N3, N1> visionMeasurementStdDevs) {
    Pose2d visionRobotPoseMeters = estimate.pose;
    double timestampSeconds = estimate.timestampSeconds;
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }
  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public boolean isNeutralZone() {
    return (
      (getPose().getMeasureX().gt(Inches.of(180))) &&
      (getPose().getMeasureX().lt(Constants.middle.getMeasureX().times(2)
          .minus(Inches.of(180))))
    );
  }

  /**
   * Is the robot drivetrain located on the right side of the field?
   */
  private boolean isRightSide() {
    return ((getPose().getMeasureY().lt(Constants.middle.getMeasureY())));
  }

  public static enum Zone {
    RED, BLUE,
    NEUTRAL
  }

  public static enum Side {
    RIGHT, LEFT
  }

  /**
   * Which side is the robot located within?
   */
  public Side getSide() {
    return (isRightSide() ? Side.RIGHT : Side.LEFT);
  }

  /**
   * Which zone is the robot located within?
   */
  public Zone getZone() {
    if (isNeutralZone()) return Zone.NEUTRAL;
    return (getPose().getMeasureX().lt(Constants.middle.getMeasureX()) ? Zone.BLUE : Zone.RED);
  }
}
