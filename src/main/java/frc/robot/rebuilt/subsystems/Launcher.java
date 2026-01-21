// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

public class Launcher extends GenericSubsystem {
  private Pivot Turret;
  private Arm Hood;
  private FlyWheel UpperShooter;
  private FlyWheel LowerShooter;

  /** Creates a new Launcher. */
  public Launcher() {
    super("launcher.json");
    Turret = (Pivot) devices.get("turretmotor");
    Hood = (Arm) devices.get("hoodmotor");
    UpperShooter = (FlyWheel) devices.get("uppershootermotor");
    LowerShooter = (FlyWheel) devices.get("lowershootermotor");
  }

  public void runShooter(double speed) {
    UpperShooter.getMotor().setDutyCycle(speed);
    LowerShooter.getMotor().setDutyCycle(speed);
  }

  public void setUpperSpeed(double speed) {
    UpperShooter.getMotor().setDutyCycle(speed);
  }

  public void setLowerSpeed(double speed) {
    LowerShooter.getMotor().setDutyCycle(speed);
  }

  public void setHoodAngle(Angle angle) {
    Hood.getMotorController().setPosition(angle);
  }

  public void setTurretRotation(Angle angle) {
    Turret.getMotorController().setPosition(angle);
  }

  public Command testLauncherCommand(double speed, double time) {

    return (Commands.run(
                () -> {
                  runShooter(speed);
                })
            .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      runShooter(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setUpperSpeed(speed);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setUpperSpeed(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setLowerSpeed(speed);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setLowerSpeed(0);
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setHoodAngle(Units.Degrees.of(90));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setHoodAngle(Units.Degrees.of(180));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setHoodAngle(Units.Degrees.of(-90));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setHoodAngle(Units.Degrees.of(-180));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setHoodAngle(Units.Degrees.of(0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setTurretRotation(Units.Degrees.of(90.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setTurretRotation(Units.Degrees.of(180.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setTurretRotation(Units.Degrees.of(-90.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setTurretRotation(Units.Degrees.of(180.0));
                    }))
                .withTimeout(time))
        .andThen(
            (Commands.run(
                    () -> {
                      setTurretRotation(Units.Degrees.of(0));
                    }))
                .withTimeout(time))
        .repeatedly();
  }

  public void ConfigController(Controller controller) {
    controller.createLeftStickButton().whileTrue(testLauncherCommand(4, 1));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}
