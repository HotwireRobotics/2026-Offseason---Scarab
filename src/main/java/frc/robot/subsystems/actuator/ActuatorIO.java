package frc.robot.subsystems.actuator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ActuatorIO {

  // Define inputs for the actuator subsystem.
  @AutoLog
  public class ActuatorInputs {
    /** Measured leader position (rotations). */
    public double position = 0.0;

    /** Commanded leader position (rotations). */
    public double target = 0.0;
  }

  /**
   * Collect the measured actuator position.
   */
  public void updateInputs(ActuatorInputs inputs);

  /**
   * Set the closed-loop target for the mechanism. The real controller closes the
   * loop on the leader motor; the simulation uses this to interpolate its position.
   *
   * @param target leader position.
   */
  public void setTarget(Angle target);
}
