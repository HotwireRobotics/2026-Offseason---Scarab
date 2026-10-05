package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface WristIO {

  // Define inputs for the actuator subsystem.
  @AutoLog
  public class ActuatorInputs {
    
  }

  /**
   * Collect inputs from the actuator's CANcoder.
   */
  public void updateInputs(ActuatorInputs inputs);

  /**
   * Set the closed-loop target for the mechanism. The real controller closes the
   * loop on the CANcoder; the simulation uses this to interpolate its position.
   *
   * @param target CANcoder-referenced target position.
   */
  public void setTarget(Angle target);
}
