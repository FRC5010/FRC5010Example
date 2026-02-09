// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class Intake extends GenericSubsystem {
  private Indexer indexer;
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {
    super("intake.json");
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(devices);
    } else {
      io = new IntakeIOReal(devices);
    }
  }

  public void runSpintake(double speed) {
    io.runSpintake(speed);
  }

  public Command spintakeCommand(double speed) {
    return Commands.run(
            () -> {
              runSpintake(speed);
            })
        .finallyDo(
            () -> {
              runSpintake(0);
            });
  }

  public void setHopperAngle(Angle angle) {
    io.setHopperAngle(angle);
  }

  public Boolean isRetracted() {
    return io.isRetracted();
  }

  public void runHopper(double speed) {
    io.runHopper(speed);
  }

  public void configTestControls(Controller controller) {
    controller
        .createRightBumper()
        .whileTrue(indexer.spindexerCommand(0.5).andThen(spintakeCommand(0.5)));
    controller.createYButton().whileTrue(getHopperSysIdCommand());
    controller.setRightYAxis(controller.createRightYAxis());
    Trigger rightYAxis = new Trigger(() -> controller.getRightYAxis() > 0.25);
    rightYAxis.onTrue(Commands.run(() -> runHopper(0.5)));
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isRequested(IntakeState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(IntakeState state) {
    return inputs.stateCurrent == state;
  }

  public void setCurrentState(IntakeState state) {
    inputs.stateCurrent = state;
  }

  public Command getHopperSysIdCommand() {
    return io.getHopperSysIdCommand();
  }

  public void setRequestedState(IntakeState state) {
    inputs.stateRequested = state;
  }
}
