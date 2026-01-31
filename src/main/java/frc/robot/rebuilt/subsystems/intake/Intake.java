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
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.velocity.FlyWheel;

public class Intake extends GenericSubsystem {
  private FlyWheel spintake;
  private Elevator intakePinion;
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {
    super("intake.json");
    spintake = (FlyWheel) devices.get("spintake");
    intakePinion = (Elevator) devices.get("pinion");
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(devices);
    } else {
      io = new IntakeIOReal(devices);
    }
  }

  public void RunSpintake(double speed) {
    spintake.set(speed);
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

  public void setPinionPosition(double position) {
    Distance mydist = Meters.of(position);
    intakePinion.setHeight(mydist);
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
