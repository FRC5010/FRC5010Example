package frc.robot.rebuilt.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {}

  public default void updateInputs(ClimbIOInputs inputs) {}
}
