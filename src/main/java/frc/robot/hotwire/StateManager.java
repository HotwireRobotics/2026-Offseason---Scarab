package frc.robot.hotwire;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class StateManager<T extends Enum<T>> {

  private T state;
  private final String name;

  public StateManager(String name, T defaultState) {
    this.name = name;
    this.state = defaultState;

    // Publish the starting state; set() only records changes.
    Logger.recordOutput(name + "/State", state.toString());
  }

  public void set(T newState) {
    if (newState != state) {
      state = newState;
      Logger.recordOutput(name + "/State", state.toString());
    }
  }

  public Command tag(T newState) {
    return Commands.runOnce(() -> set(newState));
  }

  public Command tag(Supplier<T> newStateSupplier) {
    return Commands.run(() -> set(newStateSupplier.get()));
  }

  public boolean is(T query) {
    return state == query;
  }

  public T get() {
    return state;
  }
}