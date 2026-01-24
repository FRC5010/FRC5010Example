package frc.robot.rebuilt.commands;

import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class TestCommands {

  private Map<String, GenericSubsystem> subsystems;

  Indexer indexer;
  Climb climb;
  Intake intake;
  Launcher launcher;

  public TestCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    indexer = (Indexer) subsystems.get(Constants.INDEXER);
    climb = (Climb) subsystems.get(Constants.CLIMB);
    intake = (Intake) subsystems.get(Constants.INTAKE);
    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
  }

  public void configureButtonBindings(Controller controller) {

    indexer.ConfigController(controller);
    intake.ConfigController(controller);
    climb.ConfigController(controller);
  }
}
