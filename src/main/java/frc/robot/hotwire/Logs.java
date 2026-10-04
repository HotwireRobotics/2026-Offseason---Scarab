package frc.robot.hotwire;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.PoundFeet;
import static edu.wpi.first.units.Units.PoundFoot;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.MotorIO.Setpoint;

import org.littletonrobotics.junction.Logger;

public class Logs {
  /**
   * Log TalonFX data to the Logger. This should be called periodically in the subsystem's periodic
   * method.
   *
   * @param motor
   */
  public static void log(Object fix, Motor motor) {
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Position",
        motor.getPosition().in(Rotations),
        "rotations");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Velocity",
        motor.getVelocity().in(RotationsPerSecond),
        "RPS");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Acceleration",
        motor.getAcceleration().in(RotationsPerSecondPerSecond),
        "RPS/s");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/SupplyVoltage",
        motor.getVoltage().in(Volts),
        "V");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/SupplyCurrent",
        motor.getCurrent().in(Amps),
        "A");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/StatorCurrent",
        motor.getStator().in(Amps),
        "A");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Temperature",
        motor.getTemperature().in(Celsius),
        "C");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Torque",
        motor.getTorque().in(PoundFoot),
        "lb-ft");

    // Log setpoint if it exists. An empty Optional means the motor is not
    // being commanded; the reference is never null.
    Optional<Setpoint> setpoint = motor.getSetpoint();
    if (setpoint.isEmpty()) return;

    if (!setpoint.get().getVelocity().isEmpty()) {
      Logger.recordOutput(
          "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/VelocitySetpoint",
          setpoint.get().getVelocity().get().in(RotationsPerSecond),
          "RPS");
    }
    if (!setpoint.get().getPosition().isEmpty()) {
      Logger.recordOutput(
          "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/PositionSetpoint",
          setpoint.get().getPosition().get().in(Rotations),
          "rotations");
    }
  }

  public static void log(Motor motor) {
    log(motor.getID(), motor);
  }

  /**
   * Log subsystem state to the Logger. This should be called periodically in the subsystem's
   * periodic method.
   *
   * @param subsystem
   * @param state
   */
  public static void log(Subsystem subsystem, Object state) {
    Logger.recordOutput("States/" + subsystem.getName() + '/', state.toString());
  }

  /**
   * Log any key-value pair to the Logger. This can be used for logging miscellaneous data that
   * doesn't fit into the other categories. This should be called periodically in the subsystem's
   * periodic method.
   *
   * @param key
   * @param value
   */
  public static void write(String key, Object value) {
    Logger.recordOutput(key, value.toString());
  }
}
