package frc.robot.applicable.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.applicable.simulation.mechanisms.Render;
import frc.robot.constants.Constants;

public class Handler {
  private static final double FIELD_LENGTH_METERS = 16.51;
  private static final double FIELD_WIDTH_METERS = 8.04;

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(34);
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(34);

  // Hopper count.
  private int counter = 0;
  private Angle motion = Degrees.of(0);
  private Angle pitch = Degrees.of(0);
  private final int limit = 28;
  // Declare supplier for shooting.
  private final Supplier<AngularVelocity> velocity;
  private final BooleanSupplier doShoot;
  private final BooleanSupplier doIntake;
  private final Supplier<Angle> target;
  private final Supplier<Translation3d> actuator;

  // Drive suppliers.
  private final Supplier<Pose2d> pose;
  private final Consumer<Pose2d> supp;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final Gamepiece gamepieceSimulation;
  private final Render model;

  private final RobotCollisionPhysics physics;

  public Handler(
      Supplier<AngularVelocity> velocity,
      BooleanSupplier shooter,
      BooleanSupplier intake,
      Supplier<Angle> wrist,
      Supplier<Translation3d> actuator,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> chassisSpeeds,
      Consumer<Pose2d> supp) {
    this.velocity = velocity;

    this.supp = supp;

    this.actuator = actuator;

    doIntake = () -> {
      return intake.getAsBoolean() && (counter < limit) && (Math.random() > 0.99) && (pitch.lte(Degrees.of(3)));
    };

    target = wrist;

    doShoot = shooter;

    this.pose = pose;
    this.chassisSpeeds = chassisSpeeds;

    model = new Render();
    gamepieceSimulation = new Gamepiece();
    gamepieceSimulation.spawnStartingFuel();

    physics = new RobotCollisionPhysics(
        ROBOT_WIDTH_WITH_BUMPERS,
        ROBOT_LENGTH_WITH_BUMPERS);

    // Register a robot for collision with fuel.
    gamepieceSimulation.registerRobot(
        Inches.of(35),
        Inches.of(35),
        Inches.of(4),
        this.pose, this.chassisSpeeds);

    gamepieceSimulation.registerIntake(
        Inches.of(17.5), Inches.of(24.118), Inches.of(-14.5), Inches.of(15.5), doIntake, this::intake);

    gamepieceSimulation.setSubticks(5);
    gamepieceSimulation.setLoggingFrequency(30);
    gamepieceSimulation.enableAirResistance();
    gamepieceSimulation.start();
  }

  /** Attempt to decrement the gamepiece counter. */
  private void shoot() {
    // Random chance of not firing based on the fact that we usually only shoot ~4
    // per second.
    Time time = Constants.Tempo.getTime();
    if (((counter > 0) && ((time.in(Seconds) % ((10 / ((-50 * motion.in(Degrees)) + (3 * counter)))))
        + (Math.random() / 10)) < 0.05)) {
      gamepieceSimulation.launchFuel(lineate(velocity.get(), Constants.Shooter.kWheelRadius.get()));
      counter--;
    }
  }

  /** Attempt to increment gamepiece counter. */
  public void intake() {
    this.counter++;
  }

  /** Initialize with gamepiece(s). */
  public void setCounter(
      int count) {
    counter = count;
  }

  /** Update simulation. */
  public void tick() {
    if (doShoot.getAsBoolean())
      this.shoot();
    gamepieceSimulation.updateSim();

    Logger.recordOutput("Simulation/Score/Blue", Gamepiece.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("Simulation/Score/Red", Gamepiece.Hub.RED_HUB.getScore());
    Logger.recordOutput("Simulation/Pitch", pitch);
    Logger.recordOutput("Simulation/Motion", motion);

    motion = (pitch.minus(target.get().times(-1))).times(0.1).plus(
        (pitch.gt(Degrees.of(0)) ? Degrees.of(Math.random() * 0.03) : Degrees.of(0)));

    pitch = pitch.minus(motion);

    Pose3d robotPose3d = physics.getRobotPose3d(pose.get());
    Logger.recordOutput("Simulation/Pose", robotPose3d);
    Logger.recordOutput("Simulation/Components/Bumpers", Render.Poses.bumpers);
    Logger.recordOutput("Simulation/Components/Intake", Render.Poses.intake);
    Logger.recordOutput("RobotPose", pose.get());
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] { new Pose3d() });

    // Offset the intake by the actuator's linear extension so the deployed
    // position tracks the actuator subsystem.
    Translation3d extension = (actuator != null) ? actuator.get() : Translation3d.kZero;
    Logger.recordOutput("Components/Intake", new Pose3d[] {
        new Pose3d(
            0.1958 + extension.getX(), 0.0 + extension.getY(), 0.21 + extension.getZ(),
            new Rotation3d(
                Rotations.of(0),
                getWristPitch(),
                Rotations.of(0)))
    });
    physics.resolveFieldBoundaryCollision(pose.get(), supp);
  }

  public void restart() {
    gamepieceSimulation.clearFuel();
    gamepieceSimulation.spawnStartingFuel();

    Gamepiece.Hub.BLUE_HUB.resetScore();
    Gamepiece.Hub.RED_HUB.resetScore();
  }

  public void autonomous() {
    restart();
    setCounter(8);
  }

  public Angle getWristPitch() {
    return pitch;
  }

  private LinearVelocity lineate(AngularVelocity velocity, Distance radius) {
    return radius.times(Constants.Mathematics.TAU).per(Second).times(velocity.in(RotationsPerSecond));
  }

  /**
   * Lightweight ride model for the visualized robot.
   *
   * <p>The drivetrain already owns the robot's 2D pose. This class adds only the
   * "look": keeping the body inside the field and off the hub/trench colliders,
   * and deriving a smooth ride height, pitch, and roll from the terrain under the
   * wheels. Every visual quantity is driven toward a target with a first-order
   * critically-damped filter, so there is no bouncing, no launching off crests,
   * and no impulse spikes — it just settles.
   */
  private static class RobotCollisionPhysics {
    private static final double DT = 0.02;

    // Field terrain — the two mirrored bump lanes. Must match the fuel geometry.
    private static final double HUB_SIDE = 1.2;
    private static final double BUMP_ENTRY_X = 3.96;
    private static final double BUMP_PEAK_X = 4.61;
    private static final double BUMP_EXIT_X = 5.18;
    private static final double BUMP_HEIGHT = 0.165;
    private static final double BUMP_LOW_Y_MIN = 1.57;
    private static final double BUMP_LOW_Y_MAX = FIELD_WIDTH_METERS / 2.0 - 0.60;
    private static final double BUMP_HIGH_Y_MIN = FIELD_WIDTH_METERS / 2.0 + 0.60;
    private static final double BUMP_HIGH_Y_MAX = FIELD_WIDTH_METERS - 1.57;
    private static final double TRENCH_WIDTH = 1.265;
    private static final double TRENCH_BLOCK_WIDTH = 0.305;

    // Smoothing time constants (seconds): larger = smoother and lazier.
    private static final double HEIGHT_TIME_CONSTANT = 0.07;
    private static final double TILT_TIME_CONSTANT = 0.10;

    // Maximum body tilt on terrain.
    private static final double MAX_TILT_RAD = Math.toRadians(20);

    private final ColliderRect[] staticRectangles = {
        // Hub side walls.
        new ColliderRect(4.61 - HUB_SIDE / 2, FIELD_WIDTH_METERS / 2 - HUB_SIDE / 2, 4.61 + HUB_SIDE / 2,
            FIELD_WIDTH_METERS / 2 + HUB_SIDE / 2),
        new ColliderRect(
            FIELD_LENGTH_METERS - 4.61 - HUB_SIDE / 2,
            FIELD_WIDTH_METERS / 2 - HUB_SIDE / 2,
            FIELD_LENGTH_METERS - 4.61 + HUB_SIDE / 2,
            FIELD_WIDTH_METERS / 2 + HUB_SIDE / 2),
        // Trench blocks.
        new ColliderRect(3.96, TRENCH_WIDTH, 5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
        new ColliderRect(3.96, FIELD_WIDTH_METERS - 1.57, 5.18, FIELD_WIDTH_METERS - 1.57 + TRENCH_BLOCK_WIDTH),
        new ColliderRect(FIELD_LENGTH_METERS - 5.18, TRENCH_WIDTH, FIELD_LENGTH_METERS - 3.96,
            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
        new ColliderRect(
            FIELD_LENGTH_METERS - 5.18,
            FIELD_WIDTH_METERS - 1.57,
            FIELD_LENGTH_METERS - 3.96,
            FIELD_WIDTH_METERS - 1.57 + TRENCH_BLOCK_WIDTH)
    };

    private final double robotWidthMeters;
    private final double robotLengthMeters;

    // Smoothed visual state.
    private double heightMeters = 0.0;
    private double pitchRad = 0.0;
    private double rollRad = 0.0;

    private RobotCollisionPhysics(Distance robotWidth, Distance robotLength) {
      this.robotWidthMeters = robotWidth.in(Meters);
      this.robotLengthMeters = robotLength.in(Meters);
    }

    /**
     * First-order smoothing factor for a given time constant. Guarantees a stable,
     * overshoot-free approach toward the target each tick.
     */
    private static double approach(double timeConstant) {
      return 1.0 - Math.exp(-DT / timeConstant);
    }

    /**
     * Keeps the robot inside the field and out of the hub/trench colliders, then
     * updates the smoothed ride height and tilt.
     */
    private void resolveFieldBoundaryCollision(
        Pose2d pose, Consumer<Pose2d> poseSetter) {
      double halfLength = robotLengthMeters / 2.0;
      double halfWidth = robotWidthMeters / 2.0;

      Pose2d corrected = clampToField(pose, halfLength, halfWidth);
      corrected = resolveStaticColliders(corrected, halfLength, halfWidth);

      // Only nudge the drivetrain pose in teleop. Resetting odometry mid-path
      // would fight the autonomous follower, so leave it alone during auto.
      boolean movedByCollision = corrected.getTranslation().getDistance(pose.getTranslation()) > 1e-6;
      if (movedByCollision && !DriverStation.isAutonomousEnabled()) {
        poseSetter.accept(corrected);
      }

      updateRide(corrected);

      Logger.recordOutput("Simulation/RobotCollision/Corrected", movedByCollision);
      Logger.recordOutput("Simulation/RobotPhysics/HeightMeters", heightMeters);
      Logger.recordOutput("Simulation/RobotPhysics/PitchDeg", Math.toDegrees(pitchRad));
      Logger.recordOutput("Simulation/RobotPhysics/RollDeg", Math.toDegrees(rollRad));
      Logger.recordOutput("Simulation/RobotPhysics/OnBump",
          getTerrainHeight(corrected.getX(), corrected.getY()) > 1e-3);
    }

    /**
     * Clamps the oriented robot's center so its projected footprint stays within
     * the field walls.
     */
    private Pose2d clampToField(Pose2d pose, double halfLength, double halfWidth) {
      double heading = pose.getRotation().getRadians();
      double projectedHalfX = Math.abs(Math.cos(heading)) * halfLength + Math.abs(Math.sin(heading)) * halfWidth;
      double projectedHalfY = Math.abs(Math.sin(heading)) * halfLength + Math.abs(Math.cos(heading)) * halfWidth;

      double clampedX = MathUtil.clamp(pose.getX(), projectedHalfX, FIELD_LENGTH_METERS - projectedHalfX);
      double clampedY = MathUtil.clamp(pose.getY(), projectedHalfY, FIELD_WIDTH_METERS - projectedHalfY);
      if (clampedX == pose.getX() && clampedY == pose.getY()) {
        return pose;
      }
      return new Pose2d(clampedX, clampedY, pose.getRotation());
    }

    /**
     * Pushes the robot out of any static collider it overlaps, iterating a few
     * times so simultaneous overlaps resolve.
     */
    private Pose2d resolveStaticColliders(Pose2d pose, double halfLength, double halfWidth) {
      Pose2d corrected = pose;
      for (int pass = 0; pass < 8; pass++) {
        boolean changed = false;
        for (ColliderRect rect : staticRectangles) {
          Pose2d before = corrected;
          corrected = resolveRectangleCollision(corrected, rect, halfLength, halfWidth);
          if (before.getTranslation().getDistance(corrected.getTranslation()) > 1e-8) {
            changed = true;
          }
        }
        if (!changed) {
          break;
        }
      }
      return corrected;
    }

    /**
     * Minimum-translation push-out of the oriented robot footprint from an
     * axis-aligned collider rectangle.
     */
    private Pose2d resolveRectangleCollision(
        Pose2d pose, ColliderRect rect, double halfLength, double halfWidth) {
      double heading = pose.getRotation().getRadians();
      double projectedHalfX = Math.abs(Math.cos(heading)) * halfLength + Math.abs(Math.sin(heading)) * halfWidth;
      double projectedHalfY = Math.abs(Math.sin(heading)) * halfLength + Math.abs(Math.cos(heading)) * halfWidth;

      double left = pose.getX() - projectedHalfX;
      double right = pose.getX() + projectedHalfX;
      double bottom = pose.getY() - projectedHalfY;
      double top = pose.getY() + projectedHalfY;

      if (right <= rect.xMin || left >= rect.xMax || top <= rect.yMin || bottom >= rect.yMax) {
        return pose;
      }

      // Choose the smallest of the four axis push-outs.
      double pushLeft = rect.xMin - right;
      double pushRight = rect.xMax - left;
      double pushDown = rect.yMin - top;
      double pushUp = rect.yMax - bottom;

      double dx = 0.0;
      double dy = 0.0;
      double best = Math.abs(pushLeft);
      dx = pushLeft;
      if (Math.abs(pushRight) < best) {
        best = Math.abs(pushRight);
        dx = pushRight;
        dy = 0.0;
      }
      if (Math.abs(pushDown) < best) {
        best = Math.abs(pushDown);
        dx = 0.0;
        dy = pushDown;
      }
      if (Math.abs(pushUp) < best) {
        dx = 0.0;
        dy = pushUp;
      }

      return new Pose2d(pose.getX() + dx, pose.getY() + dy, pose.getRotation());
    }

    /**
     * Derives target ride height and tilt purely from the terrain under each wheel,
     * then eases the smoothed state toward it. On flat ground the target is dead
     * flat, so driving never induces wobble.
     */
    private void updateRide(Pose2d pose) {
      double heading = pose.getRotation().getRadians();
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      double halfLength = robotLengthMeters / 2.0;
      double halfWidth = robotWidthMeters / 2.0;

      // Forward and left body-axis offsets to the four wheel contact points.
      double fx = cos * halfLength;
      double fy = sin * halfLength;
      double lx = -sin * halfWidth;
      double ly = cos * halfWidth;
      double cx = pose.getX();
      double cy = pose.getY();

      double frontLeft = getTerrainHeight(cx + fx + lx, cy + fy + ly);
      double frontRight = getTerrainHeight(cx + fx - lx, cy + fy - ly);
      double rearLeft = getTerrainHeight(cx - fx + lx, cy - fy + ly);
      double rearRight = getTerrainHeight(cx - fx - lx, cy - fy - ly);

      double frontAvg = (frontLeft + frontRight) * 0.5;
      double rearAvg = (rearLeft + rearRight) * 0.5;
      double leftAvg = (frontLeft + rearLeft) * 0.5;
      double rightAvg = (frontRight + rearRight) * 0.5;

      double targetHeight = (frontLeft + frontRight + rearLeft + rearRight) * 0.25;
      double targetPitch = MathUtil.clamp(
          -Math.atan2(frontAvg - rearAvg, robotLengthMeters), -MAX_TILT_RAD, MAX_TILT_RAD);
      double targetRoll = MathUtil.clamp(
          Math.atan2(leftAvg - rightAvg, robotWidthMeters), -MAX_TILT_RAD, MAX_TILT_RAD);

      // Critically-damped ease toward the targets: smooth, no overshoot.
      heightMeters += (targetHeight - heightMeters) * approach(HEIGHT_TIME_CONSTANT);
      pitchRad += (targetPitch - pitchRad) * approach(TILT_TIME_CONSTANT);
      rollRad += (targetRoll - rollRad) * approach(TILT_TIME_CONSTANT);
    }

    /**
     * Terrain height at a field point: zero everywhere except on the two mirrored
     * bump lanes.
     */
    private double getTerrainHeight(double xMeters, double yMeters) {
      boolean onLowLane = yMeters >= BUMP_LOW_Y_MIN && yMeters <= BUMP_LOW_Y_MAX;
      boolean onHighLane = yMeters >= BUMP_HIGH_Y_MIN && yMeters <= BUMP_HIGH_Y_MAX;
      if (!onLowLane && !onHighLane) {
        return 0.0;
      }

      double blueBump = triangularBump(xMeters, BUMP_ENTRY_X, BUMP_PEAK_X, BUMP_EXIT_X, BUMP_HEIGHT);
      double redBump = triangularBump(
          xMeters,
          FIELD_LENGTH_METERS - BUMP_EXIT_X,
          FIELD_LENGTH_METERS - BUMP_PEAK_X,
          FIELD_LENGTH_METERS - BUMP_ENTRY_X,
          BUMP_HEIGHT);
      return Math.max(blueBump, redBump);
    }

    /** Piecewise-linear triangular bump rising x1 -> x2 and falling x2 -> x3. */
    private double triangularBump(double x, double x1, double x2, double x3, double peak) {
      if (x <= x1 || x >= x3) {
        return 0.0;
      }
      if (x < x2) {
        return peak * (x - x1) / (x2 - x1);
      }
      return peak * (x3 - x) / (x3 - x2);
    }

    /** Composes the 2D odometry pose with the smoothed ride height and tilt. */
    private Pose3d getRobotPose3d(Pose2d pose) {
      return new Pose3d(
          pose.getX(),
          pose.getY(),
          heightMeters,
          new Rotation3d(rollRad, pitchRad, pose.getRotation().getRadians()));
    }

    /** Axis-aligned rectangle collider in field coordinates. */
    private static class ColliderRect {
      private final double xMin;
      private final double yMin;
      private final double xMax;
      private final double yMax;

      private ColliderRect(double xMin, double yMin, double xMax, double yMax) {
        this.xMin = xMin;
        this.yMin = yMin;
        this.xMax = xMax;
        this.yMax = yMax;
      }
    }
  }
}
