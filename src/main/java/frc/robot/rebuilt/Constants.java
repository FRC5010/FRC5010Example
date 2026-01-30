package frc.robot.rebuilt;

import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;

public class Constants {
  public static final String INDEXER = Indexer.class.getSimpleName();
  public static final String CLIMB = Climb.class.getSimpleName();
  public static final String INTAKE = Intake.class.getSimpleName();
  public static final String LAUNCHER = Launcher.class.getSimpleName();

  public static class LauncherConstants {
    public static final double UPPER_SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double LOWER_SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 1.0;
    public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 1.0;
  }
}
