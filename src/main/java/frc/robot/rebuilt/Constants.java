package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.units.measure.Distance;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;

public class Constants {
  public static final String INDEXER = Indexer.class.getSimpleName();
  public static final String CLIMB = Climb.class.getSimpleName();
  public static final String INTAKE = Intake.class.getSimpleName();
  public static final String LAUNCHER = Launcher.class.getSimpleName();

  public static class ClimbConstants {
    public static final Distance MAX = Inch.of(27);
  }

  public static class LauncherConstants {
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 1.0;
    public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 1.0;
  }
}
