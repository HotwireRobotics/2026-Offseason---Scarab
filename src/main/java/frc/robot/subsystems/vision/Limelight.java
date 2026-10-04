package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LimelightHelpers;
import frc.robot.constants.LimelightHelpers.PoseEstimate;

public class Limelight implements VisionIO {

    /** List of cameras. */
    private final List<Camera> cameras;

    // Poses for each camera.
    public static class Poses {
      private static final Pose3d gamma = 
        new Pose3d(
          Meters.of(-0.0254),
          Meters.of(-0.3048), 
          Meters.of(0.503656),
          new Rotation3d(
            Degrees.of(0), 
            Degrees.of(-1), 
            Degrees.of(90)
          ));
          
      private static final Pose3d alpha = 
        new Pose3d(
          Meters.of(0.327025),
          Meters.of(0), 
          Meters.of(0.2413),
          new Rotation3d(
            Degrees.of(0), 
            Degrees.of(25), 
            Degrees.of(0)
          ));

      private static final Pose3d beta = 
        new Pose3d(
          Meters.of(-0.0254),
          Meters.of(0.3048), 
          Meters.of(0.503656),
          new Rotation3d(
            Degrees.of(0), 
            Degrees.of(1), 
            Degrees.of(-90)
          ));
    }

    /** Define assist mode for the internal IMU. */
    public static enum IMUMode {
      OFF(0),
      SEED(1),
      INTERNAL(2),
      MT1ASSIST(3),
      EXTERNAL(4);

      public final int mode;

      IMUMode(int mode) {
        this.mode = mode;
      }

      public int get() {
        return this.mode;
      }
    }
    
    /** Define localization mode. */
    private static enum Localization {
      MEGATAG1, MEGATAG2
    }

    /** Define pipeline for vision. */
    private static class Pipeline {
      /** Localization pipeline. */
      Localization localization = Localization.MEGATAG1;
      /** Pipeline index. */
      int index = 0;
      /** IMU mode. */
      IMUMode mode = IMUMode.OFF;
      // TODO: Add other vision settings (exposed in the web interface).

      /**
       * Supply pipeline with specified localization mode.
       * 
       * @param localization
       */
      public Pipeline withLocalization(Localization localization) {
        this.localization = localization;
        return this;
      }

      /**
       * Supply pipeline with specified IMU mode.
       * 
       * @param mode
       */
      public Pipeline withIMUMode(IMUMode mode) {
        this.mode = mode;
        return this;
      }
      
      /**
       * Supply pipeline with specified index.
       * 
       * @param index
       */
      public Pipeline withIndex(int index) {
        this.index = index;
        return this;
      }
    }

    private class Camera {

        /** String identifier. */
        private final String name;
        /** Pose object. */
        private final Pose3d pose;
        /** Processing pipeline. */
        private Pipeline pipeline;

        public Camera(
            String name, Pose3d pose, Pipeline pipeline
        ) {
          // Initialize name.
          this.name = name;
          this.pose = pose;

          // Set pipeline.
          this.pipeline = pipeline;

          // Initialize robot-relative pose.
          LimelightHelpers.setCameraPose_RobotSpace(name, 
            pose.getMeasureX().in(Meters),
            pose.getMeasureY().in(Meters),
            pose.getMeasureZ().in(Meters),
            pose.getRotation().getMeasureX().in(Degrees),
            pose.getRotation().getMeasureY().in(Degrees),
            pose.getRotation().getMeasureZ().in(Degrees)
          );

          // Set configuration.
          setIMUMode(pipeline.mode);
          
          // Set assist alpha.
          LimelightHelpers.SetIMUAssistAlpha(name, 0.005);
        }

        /**
         * Select IMU mode for configuration.
         */
        private void setIMUMode(IMUMode mode) {
          LimelightHelpers.SetIMUMode(name, mode.get());
        }

        /**
         * Provide a real camera pose estimate.
         * 
         * @return a real pose estimate.
         */
        public Measurement getMeasurement(Rotation2d rotation) {
          // Supply MegaTag2 with robot orientation.
          LimelightHelpers.SetRobotOrientation(name, rotation.getMeasure().in(Degrees), 0, 0, 0, 0, 0);

          // Select pose estimation method by localization mode.
          PoseEstimate estimation = pipeline.localization.equals(Localization.MEGATAG2)
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

          // Report an empty measurement when the camera has nothing to share, so
          // the camera still identifies itself to the subsystem.
          if (estimation == null) {
            return new Measurement()
              .withSource(name)
              .withPose(Pose2d.kZero)
              .withDistance(Meters.of(0))
              .withTagCount(0)
              .withStandardDeviation(Configuration.standardDeviation);
          }

          // Initialize a measurement object.
          Measurement measurement = new Measurement(estimation)
            .withSource(name)
            .withStandardDeviation(Configuration.standardDeviation);
          measurement.pose = new Pose2d(measurement.pose.getTranslation(), rotation);
          return measurement;
        }

        /** 
         * Get the camera pose. 
         * 
         * @return the camera pose.
         */
        public Pose3d getPose() {
          return this.pose;
        }
    }
    
    public Limelight() {
      // Generic pipeline.
      final Pipeline pipeline = new Pipeline()
        .withIndex(0)
        .withLocalization(Localization.MEGATAG1)
        .withIMUMode(IMUMode.INTERNAL);
      // Initialize cameras.
      cameras = List.of(
        // Secondary camera mounted perpendicular.
        new Camera("limelight-gamma", Poses.gamma, pipeline),
        // Primary camera mounted forward.
        new Camera("limelight-alpha", Poses.alpha, pipeline),
        // Tertiary camera mounted perpendicular.
        new Camera("limelight-beta",  Poses.beta,  pipeline)
      );
    }

    /**
     * Provide a real camera pose estimate.
     * 
     * @return a real pose estimate.
     */
    public List<Measurement> getMeasurements(Rotation2d rotation) {
      // Stream cameras to collect all pose estimates.
      return cameras.stream()
        .map(camera -> camera.getMeasurement(rotation))
        .collect(Collectors.toList());
    }

    
    /**
     * Collect inputs from all vision systems.
     * 
     * @param inputs system for logging.
     */
    public void updateInputs(VisionInputs inputs, Rotation2d rotation) {
      List<Measurement> measurements = getMeasurements(rotation);
      if ((measurements.size() > 0) && false) {
        // Stream all camera inputs.
        inputs.detecting = measurements.stream()
          .anyMatch(m -> m.count > 0);
        inputs.distances = measurements.stream().mapToDouble(m -> m.distance.in(Meters))
          .toArray(); // Note: never do this ever again; very annoying.
        inputs.estimates = measurements.stream().map(m -> m.pose)
          .toArray(Pose2d[]::new);
        inputs.count = measurements.stream().map(m -> m.count)
          .reduce(0, Integer::sum);
      } else {
        inputs.detecting = false;
        inputs.distances = new double[] {-1.0};
        inputs.estimates = new Pose2d[] {Pose2d.kZero};
        inputs.count = 0;
      }
    }
}
