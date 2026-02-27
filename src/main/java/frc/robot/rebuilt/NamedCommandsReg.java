package frc.robot.rebuilt;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.commands.ClimbCommands;
import frc.robot.rebuilt.commands.IndexerCommands;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.LauncherCommands;

public class NamedCommandsReg {

  public static void createNamedCommands() {
    // Launcer
    NamedCommands.registerCommand("launcherPrep", LauncherCommands.shouldPrepCommand());
    NamedCommands.registerCommand("launcherPreset", LauncherCommands.shouldPresetCommand());
    NamedCommands.registerCommand("launcherLow", LauncherCommands.shouldLowCommand());
    NamedCommands.registerCommand("launcherIdle", LauncherCommands.shouldIdleCommand());
    // intake
    NamedCommands.registerCommand("intakeIntake", IntakeCommands.shouldIntaking());
    NamedCommands.registerCommand("intakeOuttake", IntakeCommands.shouldOuttaking());
    NamedCommands.registerCommand("intakeRetracted", IntakeCommands.shouldRetracted());
    NamedCommands.registerCommand("intakeRetracting", IntakeCommands.shouldRetracting());
    // climb
    NamedCommands.registerCommand("climbDescend", ClimbCommands.shouldDescendCommand());
    NamedCommands.registerCommand("climbElevate", ClimbCommands.shouldElevateCommand());
    NamedCommands.registerCommand("climbEnable", ClimbCommands.shouldEnableCommand());
    NamedCommands.registerCommand("climbStop", ClimbCommands.shouldStopCommand());
    // indexer
    NamedCommands.registerCommand("indexerChurn", IndexerCommands.shouldChurnCommand());
    NamedCommands.registerCommand("indexerIdle", IndexerCommands.shouldIdleCommand());
    NamedCommands.registerCommand("indexerFeed", IndexerCommands.shouldFeedCommand());
  }
 
      // Launcher
  public static Command launcherPrep() { return LauncherCommands.shouldPrepCommand(); }
  public static Command launcherPreset() { return LauncherCommands.shouldPresetCommand(); }
  public static Command launcherLow() { return LauncherCommands.shouldLowCommand(); }
  public static Command launcherIdle() { return LauncherCommands.shouldIdleCommand(); }

  // Intake
  public static Command intakeIntake() { return IntakeCommands.shouldIntaking(); }
  public static Command intakeOuttake() { return IntakeCommands.shouldOuttaking(); }
  public static Command intakeRetracted() { return IntakeCommands.shouldRetracted(); }
  public static Command intakeRetracting() { return IntakeCommands.shouldRetracting(); }

  // Climb
  public static Command climbDescend() { return ClimbCommands.shouldDescendCommand(); }
  public static Command climbElevate() { return ClimbCommands.shouldElevateCommand(); }
  public static Command climbEnable() { return ClimbCommands.shouldEnableCommand(); }
  public static Command climbStop() { return ClimbCommands.shouldStopCommand(); }

  // Indexer
  public static Command indexerChurn() { return IndexerCommands.shouldChurnCommand(); }
  public static Command indexerIdle() { return IndexerCommands.shouldIdleCommand(); }
  public static Command indexerFeed() { return IndexerCommands.shouldFeedCommand(); }
      
  
}
