package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public double upperSpeedDesired = 0.0;
    public double lowerSpeedDesired = 0.0;
    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);

    public double upperSpeedActual = 0.0;
    public double lowerSpeedActual = 0.0;
    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);

    public boolean upperSpeedAtGoal = false;
    public boolean lowerSpeedAtGoal = false;
    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;

    public double upperSpeedError = 0.0;
    public double lowerSpeedError = 0.0;
    public double hoodAngleError = 0.0;
    public double turretAngleError = 0.0;

    public double hoodVelocity = 0.0;
    public double turretVelocity = 0.0;
    public double upperMotorOutput = 0.0;
    public double lowerMotorOutput = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public void runShooter(double speed);

  public void setUpperSpeed(double speed);

  public void setLowerSpeed(double speed);

  public void setHoodAngle(Angle angle);

  public void setTurretRotation(Angle angle);

  public default void updateSimulation() {}
}
