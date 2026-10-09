package frc.robot.subsystems.actuator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.motors.Motor;

/**
 * Real actuator IO. Reads the leader motor's encoder; the motors themselves are
 * owned by the {@link Actuator} subsystem and close their loop on it.
 */
public class Clypeus implements ActuatorIO {

  /** Leader motor, carrying the encoder. */
  private final Motor leader;

  private Angle target = Rotations.of(0);

  public Clypeus(Motor leader) {
    this.leader = leader;
  }

  @Override
  public void setTarget(Angle target) {
    this.target = target;
  }

  @Override
  public void updateInputs(ActuatorInputs inputs) {
    inputs.position = leader.getPosition().in(Rotations);
    inputs.target = target.in(Rotations);
  }
}
