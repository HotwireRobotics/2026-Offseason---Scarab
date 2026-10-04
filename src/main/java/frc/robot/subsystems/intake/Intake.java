package frc.robot.subsystems.intake;

import frc.robot.subsystems.motors.MotorIO.*;
import frc.robot.subsystems.motors.Motor.Application;
import frc.robot.subsystems.motors.Motor.Feedforward;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.hotwire.Logs;
import frc.robot.hotwire.StateManager;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;

import java.util.function.Supplier;

/**
 * <strong>Intake Subsystem</strong>
 * <p>Subsystem for controlling intake roller
 * motion.
 */
public class Intake extends SubsystemBase {
  
  // Subsystem abstraction.
  private final IntakeIO io;
  private final IntakeInputs inputs;

  // State system.
  public enum State {
    FORWARD,
    REVERSE,
    STOPPED
  }
  /** Subsystem state. */
  public final StateManager<State> manager = new StateManager<State>(
    getName(), State.STOPPED
  );

  // Initialize device representatives.
  /** Intake rollers. */
  final Motor rollers;

  public Intake(
    Trigger trigger
  ) {
    // Initialize abstraction.
    io = Constants.mode.equals(Mode.SIM) 
      ? new Simulation() 
      : new Forelegs();
    this.inputs = new IntakeInputs();

    // Configure devices.
    rollers = new Motor(this, Constants.MotorIDs.ROLLERS);
    rollers.apply(
      new Application(Direction.REVERSE, NeutralMode.COAST, Amps.of(40)));
    rollers.apply(new Feedforward(10, 0, 0));
    
    // Triggers.
    trigger
      .whileTrue(runVelocity(Constants.Intake.kSpeed))
      .onFalse(runHalt());
  }

  @Override
  public void periodic() {
    // Update subsystem inputs.
    io.updateInputs(inputs);
    Logs.log(rollers);
  }

  /** 
   * Run intake rollers at velocity. 
   * 
   * @param velocity Angular velocity to run rollers at.
   */
  private Command runVelocity(Supplier<AngularVelocity> velocity) {
    return rollers.runVelocity(velocity).alongWith(
      manager.tag(() -> (velocity.get().gt(RPM.of(0)) 
        ? State.FORWARD 
        : State.REVERSE)
      ))
      // Report the halt however the command ends, including a timeout or a
      // cancellation, so the state tracks the rollers rather than the intent.
      .finallyDo(() -> manager.set(State.STOPPED));
  }

  /** 
   * Halt intake rollers. Ends once the output is released, so a timed run
   * finishes instead of holding the scheduler forever.
   */
  private Command runHalt() {
    return rollers.runStop().alongWith(
      manager.tag(State.STOPPED));
  }

  /**
   * Run intake rollers.
   */
  public Command run() {
    return runVelocity(Constants.Intake.kSpeed);
  }

  /**
   * Stop intake rollers.
   */
  public Command stop() {
    return runHalt();
  }
}