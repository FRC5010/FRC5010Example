// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems;

import edu.wpi.first.units.measure.Angle;
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

  public void ConfigController(Controller controller) {}

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
