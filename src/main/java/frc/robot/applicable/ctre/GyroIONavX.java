package frc.robot.applicable.ctre;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  /**
   * Direction yaw is taken in. The NavX reports clockwise-positive, so the
   * stock implementation negates it to reach WPILib's counter-clockwise-positive
   * convention; on this robot that left it accumulating opposite the Pigeon 2,
   * so it is taken as reported instead. Applied to yaw position, yaw velocity
   * and the odometry samples together, so the three cannot disagree.
   */
  private static final double kYawSign = 1.0;

  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) 100.0);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(kYawSign * navX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(kYawSign * navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(kYawSign * value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
