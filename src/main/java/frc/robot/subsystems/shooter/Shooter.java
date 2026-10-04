package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.hotwire.StateManager;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.Application;
import frc.robot.subsystems.motors.Motor.Feedback;
import frc.robot.subsystems.motors.Motor.Feedforward;
import frc.robot.subsystems.motors.MotorIO.Direction;
import frc.robot.subsystems.motors.MotorIO.FollowerMode;
import frc.robot.subsystems.motors.MotorIO.NeutralMode;
import frc.robot.subsystems.shooter.WideShooter;

public class Shooter extends SubsystemBase {
    
  // Subsystem abstraction.
  private final ShooterIO io;
  private final ShooterInputs inputs;

  // State system.
  public enum State {
    SHOOTING,
    STOPPED,
  }
  /** Subsystem state. */
  public final StateManager<State> manager = new StateManager<State>(
    getName(), State.STOPPED
  );

  /** Distance to the target the regression is read at. */
  private final Supplier<Distance> distance;

  /** Held to take the velocity off the regression rather than the fixed speed. */
  private final BooleanSupplier regression;

  /** Velocity sensor debouncer. */
  private final Debouncer debouncer = new Debouncer(Constants.Shooter.kDebounce.get().in(Seconds));
  
  // Initialize device representatives.
  /** Shooting rollers. */
  final Motor shooter;
  /** Primary shooter. */
  final Motor primary;
  /** Secondary shooter. */
  final Motor secondary;
  /** Tertiary shooter. */
  final Motor tertiary;
  /** Quaternary shooter. */
  final Motor quaternary;

  // Shooting array.
  final Motor[] shooting;

  /**
   * @param trigger Runs the shooter.
   * @param regression Selects where the velocity comes from while the shooter
   *     runs: held, the regression for the current distance, and otherwise the
   *     published fixed speed. Runs nothing on its own.
   * @param distance Distance to the target the regression is read at.
   */
  public Shooter(
    Trigger trigger,
    BooleanSupplier regression,
    Supplier<Distance> distance
  ) {
    // Stash the velocity sources.
    this.distance = distance;
    this.regression = regression;

    // Initialize abstraction.
    io = Constants.mode.equals(Mode.SIM) 
      ? new Simulation() 
      : new WideShooter();
    this.inputs = new ShooterInputs();

    // Control loop.
    Feedforward feedforward = new Feedforward(0.8, 0, 0);
    Feedback    feedback    = new Feedback(0.24998, 0.12009, 0);

    // Configure devices.
    shooter = new Motor(this, Constants.MotorIDs.SHOOTER);
    shooter.apply(
      new Application(Direction.REVERSE, NeutralMode.COAST, Constants.Shooter.kCurrentLimit));
    shooter.apply(
      feedforward);
    
    // Configure shooter motors with identical settings. 
    Application forward = new Application(Direction.REVERSE, NeutralMode.COAST, Constants.Shooter.kCurrentLimit);
    Application reverse = new Application(Direction.FORWARD, NeutralMode.COAST, Constants.Shooter.kCurrentLimit);
    primary    = new Motor(this, Constants.MotorIDs.PRIMARY); 
    primary.apply(
      forward);
    secondary  = new Motor(this, Constants.MotorIDs.SECONDARY); 
    secondary.apply(
      reverse);
    tertiary   = new Motor(this, Constants.MotorIDs.TERTIARY); 
    tertiary.apply(
      reverse);
    quaternary = new Motor(this, Constants.MotorIDs.QUATERNARY); 
    quaternary.apply(
      forward);

    // Join all devices in a list for iteration.
    shooting = new Motor[] {
      primary, secondary, tertiary, quaternary};

    // Configure control loop.
    for (Motor m : shooting) {
      m.apply(feedforward);
      m.apply(feedback);
    }

    // Set follower control.
    secondary.follow(primary, FollowerMode.ALIGNED);
    tertiary.follow(primary,  FollowerMode.INVERSE);
    quaternary.follow(primary, FollowerMode.ALIGNED);


    // Triggers.
    trigger
      .whileTrue(runVelocity(this::getShootingSpeed))
      .onFalse(runHalt());
  }

  @Override
  public void periodic() {
    // Update subsystem inputs.
    io.updateInputs(inputs);
  }

  /**
   * Run velocity control on shooter motors.
   * 
   * @param velocity Target velocity.
   */
  private Command runVelocity(Supplier<AngularVelocity> velocity) {
    // Stream shooting commands.
    return Commands.parallel(Arrays.stream(shooting)
      .map(motor -> motor.runVelocity(velocity))
      .toArray(Command[]::new))
        .alongWith(shooter.runVelocity(
          () -> velocity.get().div(Constants.Shooter.kDivisor.get())))
        .alongWith(manager.tag(State.SHOOTING))
        // Report the halt however the command ends.
        .finallyDo(() -> manager.set(State.STOPPED));
  }

  /**
   * Returns the state of the shooter as a boolean defining its
   * shooting readiness.
   * 
   * @return Shooter readiness.
   */
  public boolean isReady() {
    if (primary.getSetpoint().isPresent()) {
      return debouncer.calculate(
          primary.getVelocity().isNear(primary.getSetpoint().get().getVelocity().get(), Constants.Shooter.kVelocityTolerance.get()) &&
          shooter.getVelocity().isNear(shooter.getSetpoint().get().getVelocity().get(), Constants.Shooter.kVelocityTolerance.get()));
    } else return false;
  }

  /**
   * Stop shooter motors. Ends once the outputs are released, so a timed run
   * finishes instead of holding the scheduler forever.
   */
  private Command runHalt() {
    // Stream halt commands.
    return Commands.parallel(Arrays.stream(shooting)
      .map(motor -> motor.runStop())
      .toArray(Command[]::new))
        .alongWith(shooter.runStop())
        .alongWith(manager.tag(State.STOPPED));
  }

  
  /**
   * Run the shooter, at whichever velocity the modifier selects.
   */
  public Command run() {
    return runVelocity(this::getShootingSpeed);
  }

  /**
   * Run the shooter on the regression, whatever the modifier is doing.
   */
  public Command runRegressed() {
    return runVelocity(this::getRegressedSpeed);
  }

  /**
   * Run the shooter at a supplied velocity, re-read every cycle.
   *
   * @param velocity Target velocity.
   */
  public Command runAt(Supplier<AngularVelocity> velocity) {
    return runVelocity(velocity);
  }

  /**
   * Velocity the shooter is held at. Read every cycle the shooter runs, so the
   * modifier can be taken or released, and the robot can move, mid-shot.
   *
   * @return Target velocity.
   */
  public AngularVelocity getShootingSpeed() {
    return regression.getAsBoolean()
      ? getRegressedSpeed()
      : Constants.Shooter.kSpeed.get();
  }

  /**
   * Velocity the regression gives for the current distance to the target.
   *
   * @return Regressed velocity.
   */
  public AngularVelocity getRegressedSpeed() {
    return Constants.regress(distance.get());
  }

  /**
   * Stop intake rollers.
   */
  public Command stop() {
    return runHalt();
  }

  public static class ShootingVector {
    private final Pose3d position;
    private final Translation3d velocity;

    public ShootingVector(AngularVelocity velocity, Pose3d position) {
      this.position = position;
      final double vel = velocity.in(RotationsPerSecond) * Constants.Shooter.kWheelRadius.get().in(Meters);

      // Normalize to vector.
      this.velocity = new Translation3d(
        Meters.of(Math.cos(vel)), 
        Meters.of(Math.sin(vel)), 
        Meters.of(0));
    }

    /**
     * Get shooting position.
     * 
     * @return position of shooter.
     */
    public Pose3d getPosition() {
      return position;
    }

    /**
     * Get shooting velocity vector.
     * 
     * @return velocity vector of shooter.
     */
    public Translation3d getVelocity() {
      return velocity;
    }
  }
  /**
   * Get shooting position and vector.
   */
  public ShootingVector getShootingVector() {
    return new ShootingVector(primary.getVelocity(), new Pose3d(
      Meters.of(-0.183302), Meters.of(0.31), Inches.of(14.759196),
    Rotation3d.kZero));
  }
}
