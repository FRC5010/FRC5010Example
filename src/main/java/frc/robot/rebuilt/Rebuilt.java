// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.commands.AutoCommands;
import frc.robot.rebuilt.commands.ClimbCommands;
import frc.robot.rebuilt.commands.IndexerCommands;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.commands.TestCommands;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.utils.geometry.AllianceFlipUtil;

/** This is an example robot class. */
public class Rebuilt extends GenericRobot {
  public static GenericDrivetrain drivetrain;
  public static Indexer indexer;
  public static Climb climb;
  public static Intake intake;
  public static Launcher launcher;
  public static LauncherCommands launcherCommands;
  public static AutoCommands autocommands;
  public static ClimbCommands climbCommands;
  public static IntakeCommands intakecommands;
  public static IndexerCommands indexerCommands;
  public static TestCommands testCommands;

  public Rebuilt(String directory) {
    super(directory);
    AllianceFlipUtil.configure(FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
    indexer = new Indexer();
    climb = new Climb();
    intake = new Intake();
    launcher = new Launcher();
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    testCommands = new TestCommands(subsystems);
    climbCommands = new ClimbCommands(subsystems);
    launcherCommands = new LauncherCommands(subsystems);
    intakecommands = new IntakeCommands(subsystems);
    indexerCommands = new IndexerCommands(subsystems);
    autocommands = new AutoCommands(subsystems);
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    climbCommands.configureButtonBindings(driver, operator);
    launcherCommands.configureButtonBindings(driver, operator);
    intakecommands.configureButtonBindings(driver);
    indexerCommands.configureButtonBindings(driver, operator);
  }

  @Override
  public void configureAltButtonBindings(Controller driver, Controller operator) {
    // Add test mode specific button bindings here
    testCommands.configureButtonBindings(driver);
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
    launcherCommands.setDefaultCommands();
  }

  @Override
  public void initAutoCommands() {
    drivetrain.setAutoBuilder();
  }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drivetrain.generateAutoCommand(autoCommand);
  }

  @Override
  public void buildAutoCommands() {
    super.buildAutoCommands();
    selectableCommand.addOption("Do Nothing", Commands.none());
    drivetrain.addAutoCommands(selectableCommand);
  }
}
