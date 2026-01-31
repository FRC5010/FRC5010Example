// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import frc.robot.rebuilt.subsystems.Indexer.IndexerIOInputsAutoLogged;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class Indexer extends GenericSubsystem {
  private PercentControlMotor Spindexer;
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private PercentControlMotor Feeder;

  /** Creates a new Index. */
  public Indexer() {
    super("indexer.json");

    if (RobotBase.isSimulation()) {
      io = new IndexerIOSim(devices);
    } else {
      io = new IndexerIOReal(devices);
    }
  }

  public void RunSpindexer(double speed) {
    Spindexer.set(speed);
  }

  public void RunFeeder(double speed) {
    Feeder.set(speed);
  }

  public void ConfigController(Controller controller) {
    controller.createAButton().whileTrue(spindexerCommand(.25));
    controller.createBButton().whileTrue(feederCommand(.25));
  }

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

  public Command spindexerCommand(double speed) {
    return Commands.run(
            () -> {
              RunSpindexer(0.25);
            })
        .finallyDo(
            () -> {
              RunSpindexer(0);
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
}
