package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface IntakeIO {

  // Define inputs for intake subsystem.
  @AutoLog
  public class IntakeInputs {
    // Generic logging.
  }

  /**
   * Collect inputs from all vision systems.
   */
  public void updateInputs(IntakeInputs inputs);
}
