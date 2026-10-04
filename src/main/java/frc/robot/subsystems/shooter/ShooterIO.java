package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterIO {
    
  // Define inputs for intake subsystem.
  @AutoLog
  public class ShooterInputs {
    // Generic logging.
  }

  /**
   * Collect inputs from all vision systems.
   */
  public void updateInputs(ShooterInputs inputs);
}
