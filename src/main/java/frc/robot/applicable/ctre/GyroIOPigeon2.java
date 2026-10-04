package frc.robot.applicable.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.applicable.ctre.generated.TunerConstants;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Double> noMotionCount = pigeon.getNoMotionCount();

  /**
   * Orientation of the Pigeon 2 in the robot frame, in degrees. Applying a full
   * Pigeon2Configuration below resets the device's stored MountPose, so a
   * calibration performed in Tuner X does not survive a reboot unless it is
   * repeated here.
   */
  private static final double
      kMountPoseYaw   = 0.0,
      kMountPosePitch = 0.0,
      kMountPoseRoll  = 0.0;

  /**
   * Gyro scale trim, in degrees of error per full rotation. These name the
   * Pigeon's own axes, not the robot's: yaw is whichever axis the mount pose
   * above puts vertical, which is Z for a Pigeon mounted flat. Trimming an axis
   * the robot does not rotate about has no effect on yaw.
   */
  private static final double
      kGyroScalarX = -5.1,
      kGyroScalarY = -0.41515+4.7999+5.1,
      kGyroScalarZ = -5.1;

  public GyroIOPigeon2() {
    // Every group on the device is overwritten by a full configuration, mount
    // pose and gyro trim included, so both are declared above and applied here
    // as part of it rather than left at the factory default.
    Pigeon2Configuration configuration =
        TunerConstants.DrivetrainConstants.Pigeon2Configs != null
            ? TunerConstants.DrivetrainConstants.Pigeon2Configs
            : new Pigeon2Configuration();

    configuration.MountPose
        .withMountPoseYaw(kMountPoseYaw)
        .withMountPosePitch(kMountPosePitch)
        .withMountPoseRoll(kMountPoseRoll);

    configuration.GyroTrim
        .withGyroScalarX(kGyroScalarX)
        .withGyroScalarY(kGyroScalarY)
        .withGyroScalarZ(kGyroScalarZ);

    pigeon.getConfigurator().apply(configuration);
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.noMotionCount = noMotionCount.getValueAsDouble();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
