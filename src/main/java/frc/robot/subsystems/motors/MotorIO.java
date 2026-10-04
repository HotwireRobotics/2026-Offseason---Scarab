package frc.robot.subsystems.motors;

import java.lang.StackWalker.Option;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.motors.MotorIO.Setpoint;

public interface MotorIO {

  /** Set output voltage */
  void putVoltage(Voltage volts);

  /** Stop motor */
  void stop();

  /** Update sensor inputs. */
  void updateInputs(MotorInputs inputs);

  /** Limit current. */
  void setCurrentLimit(Current limit);

  public static enum Direction {
    FORWARD,
    REVERSE
  }
  /** Set direction. */
  void setDirection(Direction direction);

  public static enum NeutralMode {
    COAST,
    BRAKE
  }
  /** Set neutral mode. */
  void setNeutralMode(NeutralMode mode);

  public static enum FollowerMode {
    ALIGNED,
    INVERSE
  }
  /** Set motor follower target. */
  void follow(Motor leader, FollowerMode mode);
  /** Set motor follower target from id. */
  void follow(int leader, FollowerMode mode);

  // Configure feedback and feedforward values.
  void configureProportional(double kP);

  void configureIntegral(double kI);

  void configureDerivative(double kD);

  void configureStaticFriction(double kS);

  void configureVelocity(double kV);

  void configureAcceleration(double kA);

  /** Run to position. */
  void putPosition(Angle position);

  /** Run to velocity. */
  void putVelocity(AngularVelocity velocity);

  /** Run to percent. */
  void putPercent(double percent);

  /** Get measured velocity. */
  AngularVelocity getVelocity();

  /** Get measured acceleration. */
  AngularAcceleration getAcceleration();

  /** Get measured position. */
  Angle getPosition();

  /** Get measured current. */
  Current getCurrent();

  /** Get measured stator current. */
  Current getStator();

  /** Get measured temperature. */
  Temperature getTemperature();

  /** Get measured voltage. */
  Voltage getVoltage();

  /** Get measured torque. */
  Torque getTorque();


  /** Setpoint */
  public static class Setpoint {
    // Provide constructors for position and velocity setpoints, and suppliers for each.
    private Angle position;
    private AngularVelocity velocity;

    public Setpoint(Angle position) {
      this.position = position;
    }

    public Setpoint(AngularVelocity velocity) {
      this.velocity = velocity;
    }

    /** Empty unless this is a position setpoint. */
    public Optional<Angle> getPosition() {
      return Optional.ofNullable(position);
    }

    /** Empty unless this is a velocity setpoint. */
    public Optional<AngularVelocity> getVelocity() {
      return Optional.ofNullable(velocity);
    }

    public static Setpoint of(Angle position) {
      return new Setpoint(position);
    }

    public static Setpoint of(AngularVelocity velocity) {
      return new Setpoint(velocity);
    }
  }
  /** Get setpoint. */
  Optional<Setpoint> getSetpoint();

  /** Get device id. */
  int getID();

  @AutoLog
  public class MotorInputs {
    /** Rotor position (rotations). */
    public double position = 0.0;

    /** Rotor velocity (rotations per second). */
    public double velocity = 0.0;

    /** Rotor acceleration (rotations per second squared). */
    public double acceleration = 0.0;

    /** Voltage applied to the motor (V). */
    public double voltage = 0.0;

    /** Supply voltage (V). */
    public double supplyVoltage = 0.0;

    /** Supply current drawn from the bus (A). */
    public double current = 0.0;

    /** Stator current inside the motor (A). */
    public double statorCurrent = 0.0;

    /** Device temperature (°C). */
    public double temperature = 0.0;

    /** Applied output as a normalized value [-1, 1]. */
    public double appliedOutput = 0.0;

    /** Duty cycle output (percentage of full output). */
    public double dutyCycle = 0.0;

    /** True if the motor controller is connected and responding. */
    public boolean connected = true;
  }
}