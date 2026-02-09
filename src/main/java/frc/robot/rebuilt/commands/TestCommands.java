package frc.robot.rebuilt.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class TestCommands {

  private Map<String, GenericSubsystem> subsystems;

  Indexer indexer;
  Climb climb;
  Intake intake;
  static Launcher launcher;

  public TestCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    indexer = (Indexer) subsystems.get(Constants.INDEXER);
    climb = (Climb) subsystems.get(Constants.CLIMB);
    intake = (Intake) subsystems.get(Constants.INTAKE);
    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
  }

  public void configureButtonBindings(Controller controller) {
    launcher.setDefaultCommand(launcher.getDefaultCommand());

    indexer.ConfigController(controller);
    intake.configTestControls(controller);
    climb.ConfigController(controller);
    controller.createLeftStickButton().whileTrue(testLauncherCommand(4, 1));
    controller
        .createBButton()
        .whileTrue(launcher.getTurretSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createAButton()
        .whileTrue(launcher.getFlyWheelSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createXButton()
        .whileTrue(launcher.getHoodSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
  }

  public static Command testLauncherCommand(double speed, double time) {

    return (Commands.run(
                () -> {
                  launcher.runShooter(speed);
                },
                launcher)
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
                      launcher.runShooter(speed);
                    }))
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
