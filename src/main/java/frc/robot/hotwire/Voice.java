package frc.robot.hotwire;

import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * <strong>Voice Interface</strong>
 * <p>Dispatches commands from utterances transcribed off-robot. The transcriber
 * publishes {@code "<nonce> <verb> <seconds>"} to the entry below, dropping the
 * seconds field for phrases that carry no duration. The nonce is what lets the
 * same phrase fire twice in a row.
 *
 * <p>Utterances arrive as dashboard inputs, so a replay dispatches whatever was
 * heard at the time.
 *
 * <p>One voice command runs at a time; a new utterance supersedes the one
 * before it. The subsystem command factories hold no requirements of their own,
 * so the scheduler will not do this for us. {@value #STOP} goes further and
 * cancels every scheduled command, including ones a button started.
 */
public class Voice {

  /** Entry the transcriber publishes to. */
  public static final String ENTRY = "/SmartDashboard/Voice/Utterance";

  /** Verb that drops everything the scheduler is running before dispatching. */
  public static final String STOP = "stop";

  // Published utterance, and the nonce of the one last dispatched.
  private final LoggedNetworkString utterance = new LoggedNetworkString(ENTRY, "");
  private String dispatched = "";

  // Bound verbs, and the command the last utterance scheduled.
  private final Map<String, Function<Optional<Time>, Command>> verbs = new HashMap<>();
  private Command active = null;

  /**
   * Bind a spoken verb to a command.
   *
   * @param verb Normalized verb the transcriber emits, such as "intake".
   * @param command Built with the spoken duration, empty when none was said.
   */
  public void bind(String verb, Function<Optional<Time>, Command> command) {
    verbs.put(verb, command);
  }

  /**
   * Dispatch a newly published utterance. Call this every cycle.
   */
  public void poll() {
    String[] fields = utterance.get().trim().split("\s+");

    // Ignore malformed utterances and ones already dispatched.
    if (fields.length < 2 || fields[0].equals(dispatched)) return;
    dispatched = fields[0];
    Logger.recordOutput("Voice/Utterance", utterance.get());

    // Resolve the verb, ignoring anything unbound.
    Function<Optional<Time>, Command> command = verbs.get(fields[1]);
    if (command == null) {
      Logger.recordOutput("Voice/Dispatched", "Unbound");
      return;
    }

    // The stop verb ends every running command, not just the last utterance.
    // poll() runs outside the scheduler's own loop, so this is safe here.
    if (fields[1].equals(STOP)) {
      CommandScheduler.getInstance().cancelAll();
      active = null;
    }

    // Supersede the running command.
    clear();
    active = command.apply(fields.length > 2 ? duration(fields[2]) : Optional.empty());
    CommandScheduler.getInstance().schedule(active);
    Logger.recordOutput("Voice/Dispatched", fields[1]);
  }

  /**
   * Cancel the running voice command, if any. Bound to disabling the robot so
   * an utterance cannot outlive the enable that heard it.
   */
  public void clear() {
    if (active != null) CommandScheduler.getInstance().cancel(active);
    active = null;
  }

  /**
   * Read a spoken duration, discarding anything unparseable.
   *
   * @param field
   */
  private static Optional<Time> duration(String field) {
    try {
      return Optional.of(Seconds.of(Double.parseDouble(field)));
    } catch (NumberFormatException exception) {
      return Optional.empty();
    }
  }
}
