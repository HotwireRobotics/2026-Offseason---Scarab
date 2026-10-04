package frc.robot.subsystems.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.motors.MotorIO.MotorInputs;
import frc.robot.subsystems.motors.MotorIO.Setpoint;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Motor {

  // Abstraction
  private final MotorIO io;
  private final MotorInputs inputs;

  private final SysIdRoutine sysIdRoutine;
  private final Subsystem subsystem;

  private Angle target = Degrees.of(0);

  public Motor(Subsystem subsystem, Integer id) {
    io = Constants.mode.equals(Mode.SIM)
      ? new Simulation(id)
      : new TalonFXIO(id);
    inputs = new MotorInputs();

    // Stash subsystem identifier.
    this.subsystem = subsystem;

    // Initialize SysId routine for this motor.
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) ->
                    Logger.recordOutput(
                        "Motors/SysId/" + this.toString(), state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.putVoltage(voltage),
                null,
                subsystem));
  }

  /** Get the enclosing subsystem for this motor. */
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Set the motor output voltage.
   *
   * @param volts
   */
  public void putVoltage(Voltage volts) {
    io.putVoltage(volts);
  }

  /**
   * Run the motor to a position.
   * 
   * @param position
   */
  public void putPosition(Angle position) {
    io.putPosition(position);
    target = position;
  }

  // Command builders.
  /**
   * Run the motor to a position. The request is left in place when the command
   * ends, so the mechanism holds rather than dropping.
   */
  public Command runPosition(Angle position) {
    return Commands.run(() -> putPosition(position));
  }

  /**
   * Run the motor to a supplied position, re-read every cycle. Accepts a
   * {@link frc.robot.hotwire.Tunable} directly, so a dashboard slider moves the
   * motor while the command runs.
   */
  public Command runPosition(Supplier<Angle> position) {
    return Commands.run(() -> putPosition(position.get()));
  }
  
  /**
   * Run the motor to a velocity, neutralizing the output when the command ends.
   * Phoenix control requests persist until superseded, so without the release
   * the motor keeps spinning long after the command is done.
   */
  public Command runVelocity(AngularVelocity velocity) {
    return Commands.runEnd(() -> io.putVelocity(velocity), io::stop);
  }

  /**
   * Run the motor to a supplied velocity, re-read every cycle. Accepts a
   * {@link frc.robot.hotwire.Tunable} directly, so a dashboard slider moves the
   * motor while the command runs. Releases the output as above.
   */
  public Command runVelocity(Supplier<AngularVelocity> velocity) {
    return Commands.runEnd(() -> io.putVelocity(velocity.get()), io::stop);
  }

  /**
   * Run the motor at a percent output, neutralizing it when the command ends.
   */
  public Command runPercent(double percent) {
    return Commands.runEnd(() -> io.putPercent(percent), io::stop);
  }

  /**
   * Neutralize the motor and finish. Ends on the cycle it is scheduled, unlike
   * the run builders above, which hold their output until cancelled.
   */
  public Command runStop() {
    return Commands.runOnce(io::stop);
  }

  /**
   * Return target position.
   * 
   * @param position
   */
  public Angle getTargetPose() {
    return target;
  }

  /**
   * Run the motor at a velocity.
   * 
   * @param velocity
   */
  public void putVelocity(AngularVelocity velocity) {
    io.putVelocity(velocity);
  }

  /**
   * Run percent voltage output.
   * 
   * @param percent
   */
  public void putPercent(double percent) {
    io.putPercent(percent);
  }

  /**
   * Stop the motor.
   */
  public void stop() {
    io.stop();
  }

  /** Feedforward groups. */
  public static class Feedforward {
    public double kP, kI, kD;

    public Feedforward(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }
  }

  /** Feedback groups. */
  public static class Feedback {
    public double kS, kV, kA;

    public Feedback(double kS, double kV, double kA) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }
  }

  /** Configure all gains */
  public void apply(Feedforward feedforward, Feedback feedback) {
    apply(feedback);
    apply(feedforward);
  }

  /** Apply all feedforward gains. */
  public void apply(Feedforward feedforward) {
    configureProportional(feedforward.kP);
        configureIntegral(feedforward.kI);
      configureDerivative(feedforward.kD);
  }

  /** Apply all feedback gains. */
  public void apply(Feedback feedback) {
     configureStaticFriction(feedback.kS);
           configureVelocity(feedback.kV);
       configureAcceleration(feedback.kA);
  }

  /**
   * Configure proportional gain for this motor.
   * 
   * @param kP
   */
  public void configureProportional(double kP) {
    io.configureProportional(kP);
  }

  /**
   * Configure integral gain for this motor.
   * 
   * @param kI
   */
  public void configureIntegral(double kI) {
    io.configureIntegral(kI);
  }

  /**
   * Configure derivative gain for this motor.
   * 
   * @param kD
   */
  public void configureDerivative(double kD) {
    io.configureDerivative(kD);
  }

  /**
   * Configure static friction feedforward for this motor.
   * 
   * @param kS
   */
  public void configureStaticFriction(double kS) {
    io.configureStaticFriction(kS);
  }

  /**
   * Configure velocity feedforward for this motor.
   * 
   * @param kV
   */
  public void configureVelocity(double kV) {
    io.configureVelocity(kV);
  }

  /**
   * Configure acceleration feedforward for this motor.
   * 
   * @param kA
   */
  public void configureAcceleration(double kA) {
    io.configureAcceleration(kA);
  }

  /** Configuration groups. */
  public static class Application {
    public MotorIO.Direction direction;
    public MotorIO.NeutralMode neutral;
    public Current limit;

    public Application(
        MotorIO.Direction direction, MotorIO.NeutralMode neutral, Current limit) {
      this.direction = direction;
      this.neutral = neutral;
      this.limit = limit;
    }
  }

  /**
   * Apply a configuration.
   * 
   * @param application
   */
  public void apply(Application application) {
    setDirection(application.direction);
    setNeutralMode(application.neutral);
    setCurrentLimit(application.limit);
  }

  /**
   * Set the current limit for this motor.
   *
   * @param limit
   */
  public void setCurrentLimit(Current limit) {
    io.setCurrentLimit(limit);
  }

  /** Set the direction for this motor. */
  public void setDirection(MotorIO.Direction direction) {
    io.setDirection(direction);
  }

  /** Set the neutral mode for this motor. */
  public void setNeutralMode(MotorIO.NeutralMode mode) {
    io.setNeutralMode(mode);
  }
  
  /** Set this motor to follow a leader motor. */
  public void follow(Motor leader, MotorIO.FollowerMode mode) {
    io.follow(leader.getID(), mode);
  }

  /** Set this motor to follow a leader motor by id. */
  public void follow(int leader, MotorIO.FollowerMode mode) {
    io.follow(leader, mode);
  }

  /**
   * Get the current velocity of the motor.
   * 
   * @return velocity
   */
  public AngularVelocity getVelocity() {
    return io.getVelocity();
  }

  /**
   * Get the current acceleration of the motor.
   * 
   * @return acceleration
   */
  public AngularAcceleration getAcceleration() {
    return io.getAcceleration();
  }

  /**
   * Get the current position of the motor.
   * 
   * @return position
   */
  public Angle getPosition() {
    return io.getPosition();
  }

  /**
   * Get the current drawn by the motor.
   * 
   * @return current
   */
  public Current getCurrent() {
    return io.getCurrent();
  }

  /**
   * Get the current stator current of the motor.
   * 
   * @return stator current
   */
  public Current getStator() {
    return io.getStator();
  }

  /**
   * Get the current temperature of the motor.
   * 
   * @return temperature
   */
  public Temperature getTemperature() {
    return io.getTemperature();
  }

  /**
   * Get the current supply voltage of the motor.
   * 
   * @return voltage
   */
  public Voltage getVoltage() {
    return io.getVoltage();
  }

  /**
   * Get the current torque of the motor.
   * 
   * @return torque
   */
  public Torque getTorque() {
    return io.getTorque();
  }

  /** Get the current setpoint of the motor. */
  public Optional<Setpoint> getSetpoint() {
    return io.getSetpoint();
  }

  /**
   * Get device id.
   */
  public int getID() {
    return io.getID();
  }

  /**
   * Log motor data to the Logger. This should be called periodically in the subsystem's periodic
   * method.
   */
  public void log() {
    io.updateInputs(inputs);

    Logger.recordOutput("Motor/Velocity", inputs.velocity);
    Logger.recordOutput("Motor/Current", inputs.current);
    Logger.recordOutput("Motor/Voltage", inputs.voltage);
  }

  /**
   * Run the quasistatic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Run the dynamic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Run all SysId routines in sequence for this motor, including both quasistatic and dynamic
   * tests in both directions.
   */
  public Command runSysId() {
    return new SequentialCommandGroup(
        sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        sysIdDynamic(SysIdRoutine.Direction.kForward),
        sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}