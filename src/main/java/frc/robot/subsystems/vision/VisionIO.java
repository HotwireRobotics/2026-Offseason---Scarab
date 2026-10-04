package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.Limelight.IMUMode;

public interface VisionIO {
  /**
   * Structure for camera pose estimates.
   */
  public static class Measurement {
    /** Pose estimate. */
    public Pose2d pose;
    /** Average estimated distance from target. */
    public Distance distance;
    /** Marks the measurement time. */
    public Time timestamp;
    /** Tag count. */
    public int count;
    /** Standard deviation. */
    public Matrix<N3, N1> devs;
    /** Raw estimation data. */
    public PoseEstimate est = new PoseEstimate();
    /** Name of the camera that produced the measurement. */
    public String source = "";

    public Measurement() {}

    public Measurement(
      PoseEstimate estimation
    ) {
      this.est = estimation;
      this.pose = estimation.pose;
      this.count = est.tagCount;
      this.distance = Meters.of(est.avgTagDist);
      this.timestamp = Seconds.of(est.timestampSeconds);
    }

    /**
     * Supply measurement with pose.
     * 
     * @param pose
     */
    public Measurement withPose(Pose2d pose) {
      est.pose = pose;
      this.pose = pose;
      return this;
    }
    
    /**
     * Supply measurement with average distance.
     * 
     * @param pose
     */
    public Measurement withDistance(Distance distance) {
      est.avgTagDist = distance.in(Meters);
      this.distance = distance;
      return this;
    }
      
    /**
     * Supply measurement with time stamp.
     * 
     * @param pose
     */
    public Measurement withTimestamp(Time time) {
      est.timestampSeconds = time.in(Seconds);
      this.timestamp = time;
      return this;
    }

    /**
     * Supply measurement with tag count.
     * 
     * @param count
     */
    public Measurement withTagCount(int count) {
      est.tagCount = count;
      this.count = count;
      return this;
    }

    /**
     * Supply measurement with the name of its source camera.
     * 
     * @param source
     */
    public Measurement withSource(String source) {
      this.source = source;
      return this;
    }

    /**
     * Supply measurement with standard deviations.
     * 
     * @param devs
     */
    public Measurement withStandardDeviation(Matrix<N3, N1> devs) {
      this.devs = devs;
      return this;
    }
  }

  /**
   * Provide camera-estimated pose.
   * 
   * @return a pose estimate if the camera detects a valid target.
   */
  public List<Measurement> getMeasurements(Rotation2d rotation);

  // Define inputs for vision subsystem.
  @AutoLog
  public class VisionInputs {
    // Generic logging.

    /** Is the camera detecting a target? */
    boolean detecting = false;

    /** Average distance between the estimated pose and a target. */
    double[] distances = new double[] {};

    /** Pose estimation by camera. */
    Pose2d[] estimates = new Pose2d[] {};

    /** Number of targets. */
    int count = 0;

    // // Limelight specific logging.

    // /** Status of the inertial measurement unit (IMU) */
    // int mode = 0;

    // /** Pipeline index. */
    // int index = 0;
  }

  /**
   * Collect inputs from all vision systems.
   */
  public void updateInputs(VisionInputs inputs, Rotation2d rotation);
}