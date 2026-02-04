package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.rebuilt.commands.LauncherCommands;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public LauncherCommands.LauncherState stateRequested = LauncherCommands.LauncherState.IDLE;
    public LauncherCommands.LauncherState stateCurrent = LauncherCommands.LauncherState.IDLE;

    public AngularVelocity flyWheelSpeedDesired = RPM.of(0.0);

    public AngularVelocity flyWheelSpeedCalculated = RotationsPerSecond.of(0.0);
    public Angle hoodAngleCalculated = Degrees.of(0.0);
    public Angle turretAngleCalculated = Degrees.of(0.0);

    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);

    public AngularVelocity flyWheelSpeedActual = RPM.of(0.0);
    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);

    public boolean flyWheelSpeedAtGoal = false;
    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;

    public AngularVelocity flyWheelSpeedError = RPM.of(0.0);
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

  public void setFlyWheelVelocity(AngularVelocity speed);

  public void setHoodAngle(Angle angle);

  public void setTurretRotation(Angle angle);

  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity);

  public default void updateSimulation() {}
}
