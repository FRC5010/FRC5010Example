package frc.robot.rebuilt;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.rebuilt.commands.ClimbCommands;
import frc.robot.rebuilt.commands.IndexerCommands;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.LauncherCommands;

public class NamedCommandsReg {

  public static void createNamedCOmmands() {
    // Launcer
    NamedCommands.registerCommand("LauncherPrep", LauncherCommands.shouldPrepCommand());
    NamedCommands.registerCommand("LauncherPreset", LauncherCommands.shouldPresetCommand());
    NamedCommands.registerCommand("LauncherLow", LauncherCommands.shouldLowCommand());
    NamedCommands.registerCommand("LauncherIdle", LauncherCommands.shouldIdleCommand());
    // intake
    NamedCommands.registerCommand("IntakeIntake", IntakeCommands.shouldIntaking());
    NamedCommands.registerCommand("IntakeOuttake", IntakeCommands.shouldOuttaking());
    NamedCommands.registerCommand("IntakeRetracted", IntakeCommands.shouldRetracted());
    NamedCommands.registerCommand("IntakeRetracting", IntakeCommands.shouldRetracting());
    // climb
    NamedCommands.registerCommand("ClimbDescend", ClimbCommands.shouldDescendCommand());
    NamedCommands.registerCommand("ClimbElevate", ClimbCommands.shouldElevateCommand());
    NamedCommands.registerCommand("ClimbEnable", ClimbCommands.shouldEnableCommand());
    NamedCommands.registerCommand("ClimbStop", ClimbCommands.shouldStopCommand());
    // indexer
    NamedCommands.registerCommand("IndexerChurn", IndexerCommands.shouldChurnCommand());
    NamedCommands.registerCommand("IndexerIdle", IndexerCommands.shouldIdleCommand());
    NamedCommands.registerCommand("IndexerFeed", IndexerCommands.shouldFeedCommand());
  }
}
