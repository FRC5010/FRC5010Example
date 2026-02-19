package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommands {

  private Map<String, GenericSubsystem> subsystems;

  public AutoCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
  }

  public void configureNamedCommands() {}

  public void configureCharacterizationCommands(LoggedDashboardChooser<Command> selectableCommand) {
    selectableCommand.addOption(
        "PRO: Intake Hopper Characterization",
        ((Intake) subsystems.get(Constants.INTAKE)).getHopperCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Hood Characterization",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getHoodCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Turret Characterization",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretCharacterizationCommand());
  }
}
