// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Climb;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Elevator;

public class Climb extends GenericSubsystem {
  /** Creates a new Climb. */
  private static Elevator climber;

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Command climberCommand(Distance height) {
    return Commands.run(
            () -> {
              setClimbHeight(height);
            })
        .finallyDo(
            () -> {
              setClimbHeight(Meters.of(0));
            });
  }

  public Command idleCommand() {
    return Commands.runOnce(
        () -> {
          io.idle();
        },
        this);
  }

  public void ConfigController(Controller controller) {
    controller.createBButton().whileTrue(climberCommand(Meters.of(.5)));
  }

  public Climb() {
    super("climb.json");
    if (RobotBase.isSimulation()) {
      io = new ClimbIOSim(devices);
    } else {
      io = new ClimbIOReal(devices);
    }
  }

  public void setClimbHeight(Distance height) {
    io.setHeight(height);
  }

  public Distance getHeight() {
    return inputs.climbHeight;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }
}
