// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class Indexer extends GenericSubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  ;

  /** Creates a new Index. */
  public Indexer() {
    super("indexer.json");

    if (RobotBase.isSimulation()) {
      io = (IndexerIO) new IndexerIOSim(devices);
    } else {
      io = new IndexerIOReal(devices);
    }
  }

  public void runSpindexer(double speed) {
    io.runSpindexer(speed);
  }

  public void runTransferBack(double speed) {
    io.runTransferBack(speed);
  }

  public void runTransferFront(double speed) {
    io.runTransferFront(speed);
  }

  public void ConfigController(Controller controller) {
    controller.createAButton().whileTrue(spindexerCommand(.25));
    // controller.createBButton().whileTrue(feederCommand(.25));
  }
  /*
  public Command feederCommand(double speed) {
    return Commands.run(
            () -> {
              RunFeeder(0.25);
            })
        .finallyDo(
            () -> {
              RunFeeder(0);
            });
  }
  */

  public Command spindexerCommand(double speed) {
    return Commands.run(
            () -> {
              runSpindexer(0.25);
            })
        .finallyDo(
            () -> {
              runSpindexer(0);
            });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }

  public boolean isRequested(IndexerState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(IndexerState state) {
    return inputs.stateCurrent == state;
  }

  public void setCurrentState(IndexerState state) {
    inputs.stateCurrent = state;
  }

  public void setRequestedState(IndexerState state) {
    inputs.stateRequested = state;
  }
}
