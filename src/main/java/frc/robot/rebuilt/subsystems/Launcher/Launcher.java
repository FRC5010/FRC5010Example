// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

public class Launcher extends GenericSubsystem {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
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
    if (RobotBase.isSimulation()) {
      io = new LauncherIOSim(devices);
    } else {
      io = new LauncherIOReal(devices);
    }
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

  public void ConfigController(Controller controller) {}

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}
