package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public double flyWheelSpeedDesired = 0.0;

    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);

    public double flyWheelSpeedActual = 0.0;
    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);

    public boolean flyWheelSpeedAtGoal = false;
    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;

    public double flyWheelSpeedError = 0.0;
    public double hoodAngleError = 0.0;
    public double turretAngleError = 0.0;

    public double hoodVelocity = 0.0;
    public double turretVelocity = 0.0;
    public double flyWheelMotorOutput = 0.0;

    public Translation2d robotToTarget = new Translation2d();

    public Distance targetDistance = Meters.of(0.0);
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public void runShooter(double speed);

  public void setHoodAngle(Angle angle);

  public void setTurretRotation(Angle angle);

  public void trackTarget();

  public default void updateSimulation() {}
}
