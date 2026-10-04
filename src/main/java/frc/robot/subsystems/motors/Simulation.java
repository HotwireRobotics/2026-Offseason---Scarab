package frc.robot.subsystems.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.PoundFoot;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

public class Simulation implements MotorIO {
  // Declare device.
  private final TalonFX motor;

  // Control loops.
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final Follower follower = new Follower(0, MotorAlignmentValue.Aligned);

  // Configurator.
  private final TalonFXConfigurator configurator;

  public Simulation(int deviceID) {
    motor = new TalonFX(deviceID);
    configurator = motor.getConfigurator();
  }

  public Simulation(int deviceID, Current currentLimit) {
    this(deviceID);
    setCurrentLimit(currentLimit);
  }

  /** Get the current setpoint of the motor. */
  private Optional<Setpoint> _target = Optional.empty();

  /** Set the motor output voltage. */
  @Override
  public void putVoltage(Voltage volts) {
    motor.setControl(voltageOut.withOutput(volts.in(Volts)));
  }

  /** Run to position. */
  @Override
  public void putPosition(Angle position) {
    motor.setControl(positionVoltage.withPosition(position));
    _target = Optional.of(Setpoint.of(position));
  }

  /** Run to velocity. */
  @Override
  public void putVelocity(AngularVelocity velocity) {
    motor.setControl(positionVoltage.withVelocity(velocity));
    _target = Optional.of(Setpoint.of(velocity));
  }

  /** Run at percent. */
  @Override
  public void putPercent(double percent) {
    motor.set(percent);
  }

  /** Stop the motor, dropping the setpoint it was holding. */
  @Override
  public void stop() {
    motor.stopMotor();
    _target = Optional.empty();
  }

  /** Configure proportional gain (kP). */
  @Override
  public void configureProportional(double kP) {
    configurator.apply(new Slot0Configs().withKP(kP));
  }

  /** Configure integral gain (kI). */
  @Override
  public void configureIntegral(double kI) {
    configurator.apply(new Slot0Configs().withKI(kI));
  }

  /** Configure derivative gain (kD). */
  @Override
  public void configureDerivative(double kD) {
    configurator.apply(new Slot0Configs().withKD(kD));
  }

  /** Configure static friction feedforward (kS). */
  @Override
  public void configureStaticFriction(double kS) {
    configurator.apply(new Slot0Configs().withKS(kS));
  }

  /** Configure velocity feedforward (kV). */
  @Override
  public void configureVelocity(double kV) {
    configurator.apply(new Slot0Configs().withKV(kV));
  }

  /** Configure acceleration feedforward (kA). */
  @Override
  public void configureAcceleration(double kA) {
    configurator.apply(new Slot0Configs().withKA(kA));
  }

  /** Set the supply current limit. */
  @Override
  public void setCurrentLimit(Current limit) {
    CurrentLimitsConfigs current = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(limit.in(Amps));
    configurator.apply(current
        .withSupplyCurrentLimitEnable(true));
  }

  /** Set the motor direction. */
  @Override
  public void setDirection(Direction direction) {
    InvertedValue value = (direction == Direction.FORWARD) 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive;
    configurator.apply(
        new MotorOutputConfigs()
            .withInverted(value));
  }

  /** Set the neutral mode. */
  @Override
  public void setNeutralMode(NeutralMode mode) {
    NeutralModeValue neutral = (mode == NeutralMode.COAST) 
      ? NeutralModeValue.Coast 
      : NeutralModeValue.Brake;
    configurator.apply(
        new MotorOutputConfigs()
            .withNeutralMode(neutral));
  }

  /** Set the motor follower target. */
  @Override
  public void follow(int id, FollowerMode mode) {
    MotorAlignmentValue value = (mode == FollowerMode.ALIGNED) 
      ? MotorAlignmentValue.Aligned
      : MotorAlignmentValue.Opposed;
    motor.setControl(follower
      .withLeaderID(id)
      .withMotorAlignment(value));
  }

  /** Set the motor follower target from id. */
  @Override
  public void follow(Motor lead, FollowerMode mode) {
    follow(lead.getID(), mode);
  }

  /** Get measured velocity. */
  @Override
  public AngularVelocity getVelocity() {
    return RPM.of(0);
  }

  /** Get measured acceleration. */
  @Override
  public AngularAcceleration getAcceleration() {
    return RotationsPerSecondPerSecond.of(0);
  }

  /** Get measured position. */
  @Override
  public Angle getPosition() {
    return Degrees.of(0);
  }

  /** Get measured current. */
  @Override
  public Current getCurrent() {
    return motor.getSimState().getSupplyCurrentMeasure();
  }

  /** Get measured stator current. */
  @Override
  public Current getStator() {
    return motor.getSimState().getTorqueCurrentMeasure();
  }

  /** Get measured temperature. */
  @Override
  public Temperature getTemperature() {
    return Celsius.of(10);
  }

  /** Get measured voltage. */
  @Override
  public Voltage getVoltage() {
    return motor.getSimState().getMotorVoltageMeasure();
  }

  /** Get measured torque. */
  @Override
  public Torque getTorque() {
    return PoundFoot.of(5.23).times(motor.getSimState().getTorqueCurrentMeasure().in(Amps));
  }

  /* Get the current setpoint of the motor. */
  @Override
  public Optional<Setpoint> getSetpoint() {
    return _target;
  }

  /** Get device id. */
  @Override
  public int getID() {
    return motor.getDeviceID();
  }

  /** Update all sensor and diagnostic inputs from the motor. */
  @Override
  public void updateInputs(MotorInputs inputs) {

    // Kinematics.
    /** Rotor position (rotations). */
    inputs.position = -1;

    /** Rotor velocity (rotations per second). */
    inputs.velocity = -1;

    /** Rotor acceleration (rotations per second squared). */
    inputs.acceleration = -1;

    // Electrical.
    /** Voltage applied to the motor (V). */
    inputs.voltage = motor.getSimState().getMotorVoltage();

    /** Supply (bus) voltage (V). */
    inputs.supplyVoltage = motor.getSimState().getSupplyCurrent();

    /** Supply current drawn from the bus (A). */
    inputs.current = motor.getSimState().getSupplyCurrent();

    /** Stator (phase) current inside the motor (A). */
    inputs.statorCurrent = motor.getSimState().getTorqueCurrent();

    // Thermal.
    /** Device temperature (°C). */
    inputs.temperature = -1;

    // Output and control.
    /** Applied output as a normalized value [-1, 1]. */
    inputs.appliedOutput = -1;

    /** Duty cycle output (percentage of full output). */
    inputs.dutyCycle = -1;

    // Diagnostics.
    /** True if the motor controller is connected and responding. */
    inputs.connected = motor.isConnected();
  }
}
