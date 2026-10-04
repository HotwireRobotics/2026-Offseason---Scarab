package frc.robot.subsystems.motors;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {

  // Declare device.
  public final TalonFX motor;

  // Control loops.
  private final VoltageOut voltageOut;
  private final PositionVoltage positionVoltage;
  private final VelocityVoltage velocityVoltage;
  private final Follower follower;

  private final Slot0Configs slot;

  // Configurator.
  private final TalonFXConfigurator configurator;
  private final MotorOutputConfigs motorOutputConfigs;

  /** Get the current setpoint of the motor. */
  private Optional<Setpoint> _target;

  public TalonFXIO(int deviceID) {
    motor = new TalonFX(deviceID);
    voltageOut = new VoltageOut(0);
    positionVoltage = new PositionVoltage(0);
    velocityVoltage = new VelocityVoltage(RPM.of(0));
    follower = new Follower(0, MotorAlignmentValue.Aligned);
    slot = new Slot0Configs();
    configurator = motor.getConfigurator();
    motorOutputConfigs = new MotorOutputConfigs();
    _target = Optional.empty();
  }

  public TalonFXIO(int deviceID, Current currentLimit) {
    this(deviceID);
    setCurrentLimit(currentLimit);
  }


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
    motor.setControl(velocityVoltage.withVelocity(velocity));
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
    configurator.apply(slot.withKP(kP));
  }

  /** Configure integral gain (kI). */
  @Override
  public void configureIntegral(double kI) {
    configurator.apply(slot.withKI(kI));
  }

  /** Configure derivative gain (kD). */
  @Override
  public void configureDerivative(double kD) {
    configurator.apply(slot.withKD(kD));
  }

  /** Configure static friction feedforward (kS). */
  @Override
  public void configureStaticFriction(double kS) {
    configurator.apply(slot.withKS(kS));
  }

  /** Configure velocity feedforward (kV). */
  @Override
  public void configureVelocity(double kV) {
    configurator.apply(slot.withKV(kV));
  }

  /** Configure acceleration feedforward (kA). */
  @Override
  public void configureAcceleration(double kA) {
    configurator.apply(slot.withKA(kA));
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
        motorOutputConfigs
            .withInverted(value));
  }

  /** Set the neutral mode. */
  @Override
  public void setNeutralMode(NeutralMode mode) {
    NeutralModeValue neutral = (mode == NeutralMode.COAST) 
      ? NeutralModeValue.Coast 
      : NeutralModeValue.Brake;
    configurator.apply(
        motorOutputConfigs
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
    return motor.getVelocity().getValue();
  }

  /** Get measured acceleration. */
  @Override
  public AngularAcceleration getAcceleration() {
    return motor.getAcceleration().getValue();
  }

  /** Get measured position. */
  @Override
  public Angle getPosition() {
    return motor.getPosition().getValue();
  }

  /** Get measured current. */
  @Override
  public Current getCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  /** Get measured stator current. */
  @Override
  public Current getStator() {
    return motor.getStatorCurrent().getValue();
  }

  /** Get measured voltage. */
  @Override
  public Voltage getVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  /** Get measured temperature. */
  @Override
  public Temperature getTemperature() {
    return motor.getDeviceTemp().getValue();
  }

  /** Get measured torque. */
  @Override
  public Torque getTorque() {
    return NewtonMeters.of(7.16).times(motor.getTorqueCurrent().getValue().in(Amps));
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
    inputs.position = motor.getPosition().getValueAsDouble();

    /** Rotor velocity (rotations per second). */
    inputs.velocity = motor.getVelocity().getValueAsDouble();

    /** Rotor acceleration (rotations per second squared). */
    inputs.acceleration = motor.getAcceleration().getValueAsDouble();

    // Electrical.
    /** Voltage applied to the motor (V). */
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();

    /** Supply (bus) voltage (V). */
    inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();

    /** Supply current drawn from the bus (A). */
    inputs.current = motor.getSupplyCurrent().getValueAsDouble();

    /** Stator (phase) current inside the motor (A). */
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();

    // Thermal.
    /** Device temperature (°C). */
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();

    // Output and control.
    /** Applied output as a normalized value [-1, 1]. */
    inputs.appliedOutput = motor.getDutyCycle().getValueAsDouble();

    /** Duty cycle output (percentage of full output). */
    inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

    // Diagnostics.
    /** True if the motor controller is connected and responding. */
    inputs.connected = motor.isConnected();
  }
}