package frc.robot.subsystems.actuator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;

/**
 * Simulated actuator IO. The real mechanism drives the leader toward a target
 * with a motor; here we emulate that by exponentially interpolating the reported
 * position toward the commanded target every loop, giving smooth, life-like
 * motion between the retracted and extended setpoints.
 */
public class Simulation implements ActuatorIO {

  // Fraction of the remaining error closed each loop.
  private static final double kRate = 0.1;

  private double position;
  private double target;

  public Simulation() {
    position = Constants.Actuator.kRetracted.get().in(Rotations);
    target = position;
  }

  @Override
  public void setTarget(Angle target) {
    this.target = target.in(Rotations);
  }

  @Override
  public void updateInputs(ActuatorInputs inputs) {
    position += (target - position) * kRate;
    inputs.position = position;
    inputs.target = target;
  }
}
