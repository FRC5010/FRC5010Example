package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator;
import org.frc5010.common.utils.geometry.AllianceFlipUtil;

/**
 * A tuning command that exposes all relevant shot parameters on the SmartDashboard so that an
 * operator can iteratively find optimal hood angle and flywheel speed values at each distance.
 *
 * <h3>Dashboard keys (all prefixed with "ShotTuning/"):</h3>
 *
 * <p><b>Read-only outputs:</b>
 *
 * <ul>
 *   <li>{@code DistanceToTarget} – current distance from the turret to the hub target (meters)
 *   <li>{@code RobotPoseX}, {@code RobotPoseY}, {@code RobotPoseRot} – current robot pose
 *   <li>{@code LookupHoodAngle} – hood angle the current lookup table would produce (degrees)
 *   <li>{@code LookupFlywheelSpeed} – flywheel speed the current lookup table would produce
 *   <li>{@code BallisticHoodAngle} – hood angle from the ballistic physics model (degrees)
 *   <li>{@code BallisticFlywheelSpeed} – flywheel speed from the ballistic model
 *   <li>{@code BallisticAvailable} – whether a ballistic config has been provided
 *   <li>{@code Firing} – true while the robot is actively firing
 *   <li>{@code ActualHoodAngle} – measured hood angle from the encoder (degrees)
 *   <li>{@code ActualTurretAngle} – measured turret angle from the encoder (degrees)
 *   <li>{@code ActualFlywheelSpeed} – measured flywheel speed from the encoder (RPM)
 *   <li>{@code DesiredHoodAngle} – commanded hood angle setpoint (degrees)
 *   <li>{@code DesiredTurretAngle} – commanded turret angle setpoint (degrees)
 *   <li>{@code DesiredFlywheelSpeed} – commanded flywheel speed setpoint (RPM)
 *   <li>{@code HoodAngleError} – hood angle error: actual − desired (degrees)
 *   <li>{@code TurretAngleError} – turret angle error: actual − desired (degrees)
 *   <li>{@code FlywheelSpeedError} – flywheel speed error: actual − desired (RPM)
 *   <li>{@code HoodAtGoal} – whether hood is within tolerance
 *   <li>{@code TurretAtGoal} – whether turret is within tolerance
 *   <li>{@code FlywheelAtGoal} – whether flywheel is within tolerance
 *   <li>{@code AllAtGoal} – whether all axes are within tolerance
 * </ul>
 *
 * <p><b>Operator inputs (writable from dashboard):</b>
 *
 * <ul>
 *   <li>{@code SetHoodAngle} – desired hood angle override (degrees)
 *   <li>{@code SetFlywheelSpeed} – desired flywheel speed override (RPM)
 *   <li>{@code UseGuessSource} – "LOOKUP", "BALLISTIC", or "MANUAL" (string chooser)
 *   <li>{@code ApplyGuess} – set true to load the guess into the set-point fields
 *   <li>{@code FireShot} – set true to spin up + fire; set false to stop
 *   <li>{@code SavePoint} – set true to save the current set-points into the live lookup tables
 * </ul>
 */
public class ShotTuningCommand extends Command {
  private static final String PREFIX = "ShotTuning/";

  private final Launcher launcher;
  private final ShotCalculator shotCalculator;

  private boolean firing = false;

  /** Guess source selector – mirrors the SmartDashboard chooser. */
  public enum GuessSource {
    LOOKUP,
    BALLISTIC,
    MANUAL
  }

  /**
   * Create a new ShotTuningCommand.
   *
   * @param launcher the launcher subsystem (the command will require it)
   */
  public ShotTuningCommand(Launcher launcher) {
    this.launcher = launcher;
    this.shotCalculator = ShotCalculator.getInstance();
    addRequirements(launcher);
  }

  // -------------------------------------------------------------------------
  // Command lifecycle
  // -------------------------------------------------------------------------

  @Override
  public void initialize() {
    firing = false;

    // Seed editable dashboard values with sensible defaults
    SmartDashboard.putNumber(PREFIX + "SetHoodAngle", 31.0);
    SmartDashboard.putNumber(PREFIX + "SetFlywheelSpeed", 220.0);
    SmartDashboard.putString(PREFIX + "UseGuessSource", GuessSource.LOOKUP.name());
    SmartDashboard.putBoolean(PREFIX + "ApplyGuess", false);
    SmartDashboard.putBoolean(PREFIX + "FireShot", false);
    SmartDashboard.putBoolean(PREFIX + "SavePoint", false);
    SmartDashboard.putBoolean(PREFIX + "Firing", false);

    launcher.stopAllMotors();
  }

  @Override
  public void execute() {
    // ----- Current distance from turret to target -----
    double distance = getCurrentDistanceMeters();
    SmartDashboard.putNumber(PREFIX + "DistanceToTarget", distance);

    // ----- Robot pose -----
    Pose2d pose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
    SmartDashboard.putNumber(PREFIX + "RobotPoseX", pose.getX());
    SmartDashboard.putNumber(PREFIX + "RobotPoseY", pose.getY());
    SmartDashboard.putNumber(PREFIX + "RobotPoseRot", pose.getRotation().getDegrees());

    // ----- Lookup table values at current distance -----
    double lookupHood = shotCalculator.getLookupHoodAngleDegrees(distance);
    double lookupFlywheel = shotCalculator.getLookupFlywheelSpeed(distance);
    SmartDashboard.putNumber(PREFIX + "LookupHoodAngle", lookupHood);
    SmartDashboard.putNumber(PREFIX + "LookupFlywheelSpeed", lookupFlywheel);

    // ----- Ballistic guess at current distance -----
    boolean ballisticAvailable = shotCalculator.getBallisticConfig() != null;
    SmartDashboard.putBoolean(PREFIX + "BallisticAvailable", ballisticAvailable);
    if (ballisticAvailable) {
      double[] ballistic = shotCalculator.getBallisticGuess(distance);
      if (ballistic != null) {
        SmartDashboard.putNumber(PREFIX + "BallisticHoodAngle", ballistic[0]);
        SmartDashboard.putNumber(PREFIX + "BallisticFlywheelSpeed", ballistic[1]);
      } else {
        SmartDashboard.putNumber(PREFIX + "BallisticHoodAngle", Double.NaN);
        SmartDashboard.putNumber(PREFIX + "BallisticFlywheelSpeed", Double.NaN);
      }
    }

    // ----- Handle "ApplyGuess" button -----
    if (SmartDashboard.getBoolean(PREFIX + "ApplyGuess", false)) {
      SmartDashboard.putBoolean(PREFIX + "ApplyGuess", false); // auto-reset
      applyGuess(distance);
    }

    // ----- Handle "SavePoint" button -----
    if (SmartDashboard.getBoolean(PREFIX + "SavePoint", false)) {
      SmartDashboard.putBoolean(PREFIX + "SavePoint", false); // auto-reset
      saveCurrentPoint(distance);
    }

    // ----- Handle firing -----
    boolean fireRequested = SmartDashboard.getBoolean(PREFIX + "FireShot", false);
    double setHood = SmartDashboard.getNumber(PREFIX + "SetHoodAngle", 31.0);
    double setFlywheel = SmartDashboard.getNumber(PREFIX + "SetFlywheelSpeed", 220.0);

    if (fireRequested) {
      firing = true;
      // Apply the manual overrides to the launcher hardware
      launcher.setHoodAngle(Degrees.of(setHood));
      launcher.setTurretRotation(Degrees.of(0)); // keep turret forward for tuning
      launcher.runShooter(setFlywheel / 300.0); // normalized duty cycle – adjust to match your max
      // If using velocity control instead:
      // launcher.usePresets(Degrees.of(setHood), Degrees.of(0), RPM.of(setFlywheel));
    } else {
      if (firing) {
        // Just stopped firing – reset
        launcher.stopAllMotors();
        firing = false;
      }
    }
    SmartDashboard.putBoolean(PREFIX + "Firing", firing);

    // ----- Actual measured values from hardware -----
    SmartDashboard.putNumber(PREFIX + "ActualHoodAngle", launcher.getHoodAngleActual().in(Degrees));
    SmartDashboard.putNumber(
        PREFIX + "ActualTurretAngle", launcher.getTurretAngleActual().in(Degrees));
    SmartDashboard.putNumber(
        PREFIX + "ActualFlywheelSpeed", launcher.getFlywheelSpeedActual().in(RPM));

    // ----- Desired setpoint values (what was commanded to hardware) -----
    SmartDashboard.putNumber(
        PREFIX + "DesiredHoodAngle", launcher.getHoodAngleDesired().in(Degrees));
    SmartDashboard.putNumber(
        PREFIX + "DesiredTurretAngle", launcher.getTurretAngleDesired().in(Degrees));
    SmartDashboard.putNumber(
        PREFIX + "DesiredFlywheelSpeed", launcher.getFlywheelSpeedDesired().in(RPM));

    // ----- Errors (actual - desired) -----
    SmartDashboard.putNumber(PREFIX + "HoodAngleError", launcher.getHoodAngleError());
    SmartDashboard.putNumber(PREFIX + "TurretAngleError", launcher.getTurretAngleError());
    SmartDashboard.putNumber(
        PREFIX + "FlywheelSpeedError", launcher.getFlywheelSpeedError().in(RPM));

    // ----- At-goal flags -----
    SmartDashboard.putBoolean(PREFIX + "HoodAtGoal", launcher.isHoodAtGoal());
    SmartDashboard.putBoolean(PREFIX + "TurretAtGoal", launcher.isTurretAtGoal());
    SmartDashboard.putBoolean(PREFIX + "FlywheelAtGoal", launcher.isFlywheelAtGoal());
    SmartDashboard.putBoolean(PREFIX + "AllAtGoal", launcher.isAtGoal());
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stopAllMotors();
    firing = false;
    SmartDashboard.putBoolean(PREFIX + "Firing", false);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled
  }

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------

  /**
   * Compute the straight-line distance from the robot's current position to the hub target,
   * accounting for alliance flip.
   */
  private double getCurrentDistanceMeters() {
    Pose2d robotPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
    Translation2d hubTarget =
        AllianceFlipUtil.apply(
            frc.robot.rebuilt.FieldConstants.Hub.topCenterPoint.toTranslation2d());
    return robotPose.getTranslation().getDistance(hubTarget);
  }

  /**
   * Copy the selected guess source's values into the "Set" fields on the dashboard so the operator
   * can immediately test them.
   */
  private void applyGuess(double distance) {
    String source = SmartDashboard.getString(PREFIX + "UseGuessSource", GuessSource.LOOKUP.name());
    GuessSource guessSource;
    try {
      guessSource = GuessSource.valueOf(source);
    } catch (IllegalArgumentException e) {
      guessSource = GuessSource.LOOKUP;
    }

    switch (guessSource) {
      case LOOKUP:
        SmartDashboard.putNumber(
            PREFIX + "SetHoodAngle", shotCalculator.getLookupHoodAngleDegrees(distance));
        SmartDashboard.putNumber(
            PREFIX + "SetFlywheelSpeed", shotCalculator.getLookupFlywheelSpeed(distance));
        break;
      case BALLISTIC:
        double[] ballistic = shotCalculator.getBallisticGuess(distance);
        if (ballistic != null) {
          SmartDashboard.putNumber(PREFIX + "SetHoodAngle", ballistic[0]);
          SmartDashboard.putNumber(PREFIX + "SetFlywheelSpeed", ballistic[1]);
        }
        break;
      case MANUAL:
        // No-op – keep whatever the operator already typed in
        break;
    }
  }

  /**
   * Save the current "Set" values into the live lookup tables at the current distance so they take
   * effect immediately for the ShotCalculator.
   */
  private void saveCurrentPoint(double distance) {
    double hood = SmartDashboard.getNumber(PREFIX + "SetHoodAngle", 31.0);
    double flywheel = SmartDashboard.getNumber(PREFIX + "SetFlywheelSpeed", 220.0);
    // Estimate a simple time-of-flight based on distance
    double tof = distance / 10.0; // rough estimate; operator can refine later
    shotCalculator.addDataPoint(distance, hood, flywheel, tof);

    // Log what was saved
    System.out.println(
        String.format(
            "[ShotTuning] Saved: dist=%.2f m, hood=%.1f°, flywheel=%.1f, tof=%.3f s",
            distance, hood, flywheel, tof));
  }

  // -------------------------------------------------------------------------
  // Static factory for convenience
  // -------------------------------------------------------------------------

  /**
   * Create a command that runs shot tuning. This is intended to be bound to a button or placed in a
   * selectable command chooser.
   *
   * @param launcher the Launcher subsystem
   * @return a command that can be scheduled
   */
  public static Command create(Launcher launcher) {
    return new ShotTuningCommand(launcher);
  }

  /**
   * Wrap the tuning command with an indexer force-feed so the operator can fire shots during
   * tuning. Press once to start tuning, cancel to stop.
   *
   * @param launcher the Launcher subsystem
   * @return a command group that tunes and can fire
   */
  public static Command createWithFeed(Launcher launcher) {
    return Commands.parallel(
        new ShotTuningCommand(launcher),
        Commands.either(
            // When FireShot is active, also force-feed the indexer
            IndexerCommands.shouldForceCommand(),
            IndexerCommands.shouldChurnCommand(),
            () -> SmartDashboard.getBoolean("ShotTuning/FireShot", false)));
  }
}
