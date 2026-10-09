package frc.robot.subsystems.actuator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.hotwire.Logs;
import frc.robot.hotwire.StateManager;
import frc.robot.subsystems.actuator.ActuatorIO.ActuatorInputs;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.Application;
import frc.robot.subsystems.motors.Motor.Feedforward;
import frc.robot.subsystems.motors.MotorIO.Direction;
import frc.robot.subsystems.motors.MotorIO.FollowerMode;
import frc.robot.subsystems.motors.MotorIO.NeutralMode;

/**
 * <strong>Actuator Subsystem</strong>
 * <p>Extends and retracts the intake by sliding it along a fixed track, on a
 * proportional loop between the two setpoints.
 */
public class Actuator extends SubsystemBase {

  // Subsystem abstraction.
  private final ActuatorIO io;
  private final ActuatorInputs inputs;

  // State system.
  public enum State {
    EXTENDED,
    RETRACTED
  }
  /** Subsystem state. */
  public final StateManager<State> manager = new StateManager<State>(
    getName(), State.RETRACTED
  );

  // Initialize device representatives.
  /** Right actuator motor; carries the encoder and leads. */
  final Motor leader;
  /** Left actuator motor; follows the right motor. */
  final Motor left;

  public Actuator(
    Trigger trigger
  ) {
    // Configure devices.
    Application configuration = new Application(
      Direction.FORWARD, NeutralMode.BRAKE, Amps.of(40));
    leader = new Motor(this, Constants.MotorIDs.ACTUATOR_RIGHT);
    leader.apply(
      configuration);
    leader.apply(
      new Feedforward(Constants.Actuator.kP, 0, 0));
    left = new Motor(this, Constants.MotorIDs.ACTUATOR_LEFT);
    left.apply(
      configuration);
    left.apply(
      new Feedforward(Constants.Actuator.kP, 0, 0));

    // The left motor mirrors the leader.
    left.follow(leader, FollowerMode.INVERSE);

    // Initialize abstraction.
    io = Constants.mode.equals(Mode.SIM)
      ? new Simulation()
      : new Clypeus(leader);
    this.inputs = new ActuatorInputs();

    // Triggers.
    trigger.onTrue(runToggle());
  }

  @Override
  public void periodic() {
    // Update subsystem inputs.
    io.updateInputs(inputs);

    // Drive the leader (the follower tracks it) toward the active setpoint.
    Angle target = getTarget();
    leader.putPosition(target);
    io.setTarget(target);

    // Log device and derived state.
    Logs.log(leader);
    Logs.log  (left);
    Logger.recordOutput("Actuator/Position", getPosition());
    Logger.recordOutput("Actuator/Target",   target);
    Logger.recordOutput("Actuator/OnTarget", isTarget());
    Logger.recordOutput("Actuator/Extension", getExtension());
    Logger.recordOutput("Components/Intake", new Pose3d[] { getComponent() });
  }

  /**
   * Extend the actuator. Finishes immediately; {@code periodic()} drives the
   * mechanism there.
   */
  public Command runExtend() {
    return Commands.runOnce(this::extend, this);
  }

  /**
   * Retract the actuator. Finishes immediately, as above.
   */
  public Command runRetract() {
    return Commands.runOnce(this::retract, this);
  }

  /**
   * Toggle the actuator. Finishes immediately, as above.
   */
  public Command runToggle() {
    return Commands.runOnce(this::toggle, this);
  }

  /**
   * Extend the actuator.
   */
  public void extend() {
    manager.set(State.EXTENDED);
  }

  /**
   * Retract the actuator.
   */
  public void retract() {
    manager.set(State.RETRACTED);
  }

  /**
   * Toggle the actuator between its extended and retracted states.
   */
  public void toggle() {
    if (manager.get() == State.EXTENDED) retract(); else extend();
  }

  /**
   * Setpoint for the current state.
   *
   * @return leader position.
   */
  public Angle getTarget() {
    return manager.get() == State.EXTENDED
      ? Constants.Actuator.kExtended .get()
      : Constants.Actuator.kRetracted.get();
  }

  /**
   * Measured leader position.
   *
   * @return leader position.
   */
  public Angle getPosition() {
    return Rotations.of(inputs.position);
  }

  /**
   * Whether the actuator is within {@link Constants.Actuator#kTolerance} of its
   * setpoint.
   *
   * @return on target.
   */
  public boolean isTarget() {
    return getPosition().isNear(getTarget(), Constants.Actuator.kTolerance.get());
  }

  /**
   * Fraction of full extension, in [0, 1], derived from the measured leader
   * position. This is smoothly interpolated in simulation.
   *
   * @return extension fraction.
   */
  public double getExtension() {
    double range = Constants.Actuator.kExtended.get()
      .minus(Constants.Actuator.kRetracted.get()).in(Rotations);
    Angle travelled = getPosition().minus(Constants.Actuator.kRetracted.get());
    return MathUtil.clamp(travelled.in(Rotations) / range, 0, 1);
  }

  /**
   * Linear displacement of the intake caused by the current actuator extension.
   * The intake travels {@link Constants.Actuator#kTravel} along a
   * {@link Constants.Actuator#kAngle} incline, moving forward and downward.
   *
   * @return intake displacement, in the robot frame.
   */
  public Translation3d getDisplacement() {
    double distance = Constants.Actuator.kTravel.get().in(Meters) * getExtension();
    double angle = Constants.Actuator.kAngle.get().in(Radians);
    return new Translation3d(
      distance * Math.cos(angle),
      0,
      -distance * Math.sin(angle));
  }

  /**
   * Intake component pose, in the robot frame, for the 3D model.
   *
   * @return intake pose.
   */
  public Pose3d getComponent() {
    return new Pose3d(
      Constants.Actuator.kOrigin.plus(getDisplacement()),
      Rotation3d.kZero);
  }
}
