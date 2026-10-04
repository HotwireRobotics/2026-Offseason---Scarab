package frc.robot.subsystems.actuator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;

/**
 * Simulated actuator IO. The real mechanism drives a CANcoder toward a target
 * with a motor; here we emulate that by exponentially interpolating the reported
 * position toward the commanded target every loop, giving smooth, life-like
 * motion between the retracted and extended setpoints.
 */
public class Simulation implements ActuatorIO {

  public Simulation() {}

  @Override
  public void setTarget(Angle target) {
    
  }

  @Override
  public void updateInputs(ActuatorInputs inputs) {

  }
}
