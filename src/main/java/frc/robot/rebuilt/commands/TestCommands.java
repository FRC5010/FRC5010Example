package frc.robot.rebuilt.commands;

import frc.robot.rebuilt.subsystems.Climb;
import frc.robot.rebuilt.subsystems.Indexer;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Launcher;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.sensors.Controller;

public class TestCommands {

  private Map<String, GenericSubsystem> subsystems;

  Indexer indexer;
  Climb climb;
  Intake intake;
  Launcher launcher;

  public void TestCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    indexer = (Indexer) subsystems.get(ConfigConstants.INDEXER);
    climb = (Climb) subsystems.get(ConfigConstants.CLIMB);
    intake = (Intake) subsystems.get(ConfigConstants.INTAKE);
    launcher = (Launcher) subsystems.get(ConfigConstants.LAUNCHER);
  }

  public void configureButtonBindings(Controller controller) {

    indexer.ConfigController(controller);
    intake.ConfigController(controller);
    climb.ConfigController(controller);
  }
}
