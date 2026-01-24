// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Elevator;

public class Intake extends GenericSubsystem {
  private PercentControlMotor Spintake;
  private Elevator Winch;
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {
    super("intake.json");
    Spintake = (PercentControlMotor) devices.get("spintake");
    Winch = (Elevator) devices.get("winch");
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(devices);
    } else {
      io = new IntakeIOReal(devices);
    }
  }

  public void RunSpintake(double speed) {
    Spintake.set(speed);
  }

  public void ConfigController(Controller controller) {
    controller.createLeftBumper().whileTrue(spintakeCommand(.25));
  }

  public Command spintakeCommand(double speed) {
    return Commands.run(
            () -> {
              RunSpintake(.25);
            })
        .finallyDo(
            () -> {
              RunSpintake(0);
            });
  }

  public void setHeight(double height) {
    Distance mydist = Meters.of(height);
    Winch.getMotorController().setPosition(mydist);
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
