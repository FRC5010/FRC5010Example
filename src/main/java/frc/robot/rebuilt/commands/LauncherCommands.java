package frc.robot.rebuilt.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class LauncherCommands {
  private Launcher launcher;
  private Map<String, GenericSubsystem> subsystems;

  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
  }

  public void configureButtonBindings(Controller controller) {
    controller.createLeftStickButton().whileTrue(testLauncherCommand(4, 1));
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
