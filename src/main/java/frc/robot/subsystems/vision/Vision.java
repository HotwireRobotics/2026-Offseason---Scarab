// Hotwire-Robotics™ : 
package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.equation.MatrixConstructor;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.LimelightHelpers.PoseEstimate;
import frc.robot.hotwire.StateManager;
import frc.robot.subsystems.vision.VisionIO.Measurement;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;

/**
 * <strong>Vision Subsystem</strong>
 * <p>Camera reads apriltag markers and supplies the drivetrain with 
 * pose estimates. The vision subsystem takes suppliers for the robot
 * location and rotation. Function also requires a pose estimate consumer.
 * 
 * @param location
 * @param rotation
 * @param estimate
 */
public class Vision extends SubsystemBase {

    // Declare orientation and location suppliers.
    private final Supplier<Pose2d> location;
    private final Supplier<Rotation2d> rotation;

    // Declare pose estimate consumer.
    private final BiConsumer<PoseEstimate, Matrix<N3, N1>> estimate;

    // Subsystem abstraction.
    private final VisionIO io;
    private final VisionInputs inputs;

    // State system.
    public enum State {
        READING,
        STOPPED
    }
    /** Subsystem state. */
    public final StateManager<State> manager = new StateManager<State>(
        getName(), State.READING
    );


    public Vision(
        Supplier<Pose2d>     location,
        Supplier<Rotation2d> rotation,
        BiConsumer<PoseEstimate, Matrix<N3, N1>> estimate
    ) {
        // Initialize abstraction
        io = Constants.mode.equals(Mode.SIM) 
            ? new Simulated() 
            : new Limelight();
        inputs = new VisionInputs();

        // Initialize orientation suppliers.
        this.location = location;
        this.rotation = rotation;

        // Initialize pose estimate consumer.
        this.estimate = estimate;
    }

    /**
     * Provides the drivetrain with a pose estimate.
     * 
     * @param pose
     */
    private void supply(PoseEstimate estimation, Matrix<N3, N1> devs) {
        estimate.accept(estimation, devs);
    }

    /**
     * Get drivetrain pose.
     * 
     * @return the drivetrain pose measurement.
     */
    private Pose2d getPose() {
        return location.get();
    }

    /**
     * Get drivetrain rotation.
     * 
     * @return the gyroscope yaw measurement.
     */
    private Rotation2d getRotation() {
        return rotation.get();
    }

    
    /**
     * Determine whether a given pose estimate is valid based on the number of detected tags and the
     * average tag distance.
     *
     * @param estimation
     * @return
     */
    public boolean isValidEstimate(Measurement estimation) {
        // Trash empty measurements.
        if (estimation == null) return false;

        // Trash invalid measurements.
        return (estimation.count > 0);
    }

    /**
     * Post a camera's final pose estimate to NetworkTables. The estimate is published
     * as a single-element array, or as an empty array when the camera has no reading.
     *
     * @param estimation
     */
    private void publish(Measurement estimation) {
        Logger.recordOutput(
            "Vision/" + estimation.source + "/Estimate",
            isValidEstimate(estimation)
                ? new Pose2d[] {estimation.pose}
                : new Pose2d[] {}
        );
    }

    public void setEnabled(boolean value) {
        this.manager.set(value ? State.READING : State.STOPPED);
    }

    @Override
    public void periodic() {
        // Update inputs.
        io.updateInputs(inputs, rotation.get());
        
        // Get a pose estimate from the camera.
        List<Measurement> measurements = io.getMeasurements(rotation.get());

        // Trash empty measurements.
        if (measurements == null) return;

        // Post the final estimate of every camera to NetworkTables.
        measurements.stream()
            .filter(m -> m != null)
            .forEach(this::publish);

        // Supply pose estimates to the drivetrain.
        measurements.stream()
            .filter(m -> isValidEstimate(m))
            .forEach(m -> supply(m.est, m.devs));
    }
}