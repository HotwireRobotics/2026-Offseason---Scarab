package frc.robot.hotwire;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * <strong>Tunable Value</strong>
 * <p>Wraps a constant in a NetworkTables entry so a dashboard slider can drive
 * it while the robot runs.
 *
 * <p>A published value only moves the mechanism where the call site reads it
 * every cycle.
 *
 * @param <M> Measure the published number is read back as.
 */
public class Tunable<M> implements Supplier<M> {

  /** Table all tunables are published under. */
  public static final String TABLE = "/SmartDashboard/Tuning/";

  // Published entry and the unit it is read back through.
  private final LoggedNetworkNumber entry;
  private final DoubleFunction<M> unit;

  /**
   * Publish a tunable measure.
   *
   * @param key Path under the tuning table, subsystem first.
   * @param value Default value, expressed in the given unit.
   * @param unit Unit constructor for the published number, such as {@code RPM::of}.
   */
  public Tunable(String key, double value, DoubleFunction<M> unit) {
    this.entry = new LoggedNetworkNumber(TABLE + key, value);
    this.unit = unit;
  }

  /**
   * Read the published value as a measure.
   *
   * @return measure
   */
  public M get() {
    return unit.apply(entry.get());
  }

  /**
   * Read the published value as a bare number, in the unit it was published in.
   *
   * @return value
   */
  public double value() {
    return entry.get();
  }

  /**
   * Overwrite the published value. Read back on the following cycle.
   *
   * @param value
   */
  public void set(double value) {
    entry.set(value);
  }

  /**
   * <strong>Unitless Tunable</strong>
   * <p>Tunable for gains, divisors, and other bare numbers.
   */
  public static class Scalar extends Tunable<Double> {

    /**
     * Publish a tunable number.
     *
     * @param key Path under the tuning table, subsystem first.
     * @param value Default value.
     */
    public Scalar(String key, double value) {
      super(key, value, number -> number);
    }
  }
}
