package frc.robot.rebuilt.commands;

import java.util.Map;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Launcher;
import yams.mechanisms.positional.Arm;

public class LauncherCommands {

  private StateMachine stateMachine;
  private State lowState;
  private State prepState;
  private State readyState;
  private Arm hood;
  private Launcher launcher;
  private Map<String, GenericSubsystem> subsystems;

  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {

    this.subsystems = subsystems;

    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
    stateMachine = new StateMachine("LauncherStateMachine");

    lowState = stateMachine.addState("LOW-SPEED", Commands.idle());
  }

  public void configureButtonBindings(Controller controller) {

    if (launcher != null) {
      launcher.setDefaultCommand(stateMachine);
    }

    controller.createLeftStickButton().whileTrue(testLauncherCommand(4, 1));

    Trigger rightBumper = controller.createRightBumper();
    Trigger leftBumper = controller.createLeftBumper();

    lowState.switchTo(prepState).when(rightBumper);
    prepState.switchTo(lowState).when(() -> !rightBumper.getAsBoolean());

    prepState.switchTo(readyState).when(leftBumper);
    readyState.switchTo(lowState).when(() -> !leftBumper.getAsBoolean());

    if (lowState != null && lowState.isScheduled()) {

      launcher.setHoodAngle(Units.Degrees.of(0));
      launcher.setLowerSpeed(0.5);
      launcher.setTurretRotation(Units.Degrees.of(0));
    }
  }

  public Command testLauncherCommand(double speed, double time) {

    return (Commands.run(
                () -> {
                  launcher.runShooter(speed);
                })
            .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.runShooter(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setUpperSpeed(speed);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setUpperSpeed(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setLowerSpeed(speed);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setLowerSpeed(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setHoodAngle(Units.Degrees.of(90));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setHoodAngle(Units.Degrees.of(180));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setHoodAngle(Units.Degrees.of(-90));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setHoodAngle(Units.Degrees.of(-180));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setHoodAngle(Units.Degrees.of(0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setTurretRotation(Units.Degrees.of(90.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setTurretRotation(Units.Degrees.of(180.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setTurretRotation(Units.Degrees.of(-90.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setTurretRotation(Units.Degrees.of(180.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      launcher.setTurretRotation(Units.Degrees.of(0));
                    }))
                .withTimeout(time))
        .repeatedly();
  }
}
