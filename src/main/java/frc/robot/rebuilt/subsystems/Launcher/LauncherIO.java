package frc.robot.rebuilt.subsystems.Launcher;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {}

  public default void updateInputs(LauncherIOInputs inputs) {}
}
