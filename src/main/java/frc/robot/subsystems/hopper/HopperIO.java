package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface HopperIO {

  // Define inputs for intake subsystem.
  @AutoLog
  public class HopperInputs {
    // Generic logging.
  }

  /**
   * Collect inputs from all vision systems.
   */
  public void updateInputs(HopperInputs inputs);
}
