package frc.robot.subsystems.actuator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.subsystems.actuator.Clypeus;

import com.andymark.jni.AM_CAN_Mag_Switch;
import com.andymark.jni.AM_CAN_Mag_Switch.AM_MagSwitchData;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <strong>Actuator Subsystem</strong>
 * <p>Deploys and retracts the intake by driving it linearly along a fixed
 * incline. Composed of two motors and a CANcoder: the right motor carries the
 * CANcoder and leads, the left motor follows it.
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
  /** Right actuator motor; carries the CANcoder and leads. */
  final Motor leader;
  /** Left actuator motor; follows the right motor. */
  final Motor left;

  // Test toggle for commanding the actuator in/out; defaults to retracted.
  private boolean toggle = false;

  public Actuator(
    Trigger trigger
  ) {
    // Initialize abstraction.
    io = Constants.mode.equals(Mode.SIM)
      ? new Simulation()
      : new Clypeus();
    this.inputs = new ActuatorInputs();

    // Configure devices.
    Application configuration = new Application(
      Direction.FORWARD, NeutralMode.BRAKE, Amps.of(40));
    leader = new Motor(this, Constants.MotorIDs.ACTUATOR_RIGHT);
    leader.apply(
      configuration);
    leader.apply(
      new Feedforward(Constants.Actuator.kP, Constants.Actuator.kI, 0));
    left = new Motor(this, Constants.MotorIDs.ACTUATOR_LEFT);
    left.apply(
      configuration);
    left.apply(
      new Feedforward(Constants.Actuator.kP, Constants.Actuator.kI, 0));

    // The left motor mirrors the leader.
    left.follow(leader, FollowerMode.INVERSE);

    // Triggers.
    trigger.onTrue(Commands.runOnce(this::toggle));
  }

  @Override
  public void periodic() {
    // Update subsystem inputs.
    io.updateInputs(inputs);

    // Drive the leader (the follower tracks it) toward the active target.
    Angle target = toggle
      ? Constants.Actuator.kExtended.get()
      : Constants.Actuator.kRetracted.get();
    
    leader.putPosition(target);
    io.setTarget(target);

    // Log device and derived state.
    Logs.log(leader);
    Logger.recordOutput("Actuator/Position", leader.getPosition());
    Logger.recordOutput("Actuator/Extension",      getExtension());
  }

  /**
   * Toggle the actuator between its extended and retracted states. Takes effect
   * immediately; {@code periodic()} drives the mechanism to the new target.
   */
  public void toggle() {
    if (toggle) retract(); else extend();
  }

  /**
   * Extend the actuator.
   */
  public void extend() {
    toggle = true;
    manager.set(State.EXTENDED);
  }

  /**
   * Retract the actuator.
   */
  public void retract() {
    toggle = false;
    manager.set(State.RETRACTED);
  }

  /**
   * Fraction of full extension, in [0, 1], derived from the measured CANcoder
   * position. This is smoothly interpolated in simulation.
   *
   * @return extension fraction.
   */
  public double getExtension() {
    double range = Constants.Actuator.kExtended.get()
      .minus(Constants.Actuator.kRetracted.get()).in(Rotations);
    Angle travelled = leader.getPosition().minus(Constants.Actuator.kRetracted.get());
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
}
