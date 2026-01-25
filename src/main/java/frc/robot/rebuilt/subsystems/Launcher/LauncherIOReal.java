// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Angle;
import frc.robot.rebuilt.Constants;
import java.util.Map;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  private Pivot Turret;
  private Arm Hood;
  private FlyWheel UpperShooter;
  private FlyWheel LowerShooter;

  public LauncherIOReal(Map<String, Object> devices) {
    this.devices = devices;
    Turret = (Pivot) devices.get("turretmotor");
    Hood = (Arm) devices.get("hoodmotor");
    UpperShooter = (FlyWheel) devices.get("uppershootermotor");
    LowerShooter = (FlyWheel) devices.get("lowershootermotor");
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.upperSpeedDesired =
        UpperShooter.getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it.in(RPM))
            .orElse(0.0);
    inputs.lowerSpeedDesired =
        LowerShooter.getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it.in(RPM))
            .orElse(0.0);
    inputs.hoodAngleDesired =
        Hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    inputs.turretAngleDesired =
        Turret.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));

    inputs.upperSpeedActual = UpperShooter.getSpeed().in(RPM);
    inputs.lowerSpeedActual = LowerShooter.getSpeed().in(RPM);
    inputs.hoodAngleActual = Hood.getAngle();
    inputs.turretAngleActual = Turret.getAngle();

    inputs.upperSpeedError = inputs.upperSpeedActual - inputs.upperSpeedDesired;
    inputs.lowerSpeedError = inputs.lowerSpeedActual - inputs.lowerSpeedDesired;
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.upperSpeedAtGoal =
        Math.abs(inputs.upperSpeedError) <= Constants.LauncherConstants.UPPER_SHOOTER_TOLERANCE_RPM;
    inputs.lowerSpeedAtGoal =
        Math.abs(inputs.lowerSpeedError) <= Constants.LauncherConstants.LOWER_SHOOTER_TOLERANCE_RPM;
    inputs.hoodAngleAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.LauncherConstants.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError)
            <= Constants.LauncherConstants.TURRET_ANGLE_TOLERANCE_DEGREES;

    inputs.hoodVelocity = Hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.turretVelocity =
        Turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.upperMotorOutput = UpperShooter.getMotor().getStatorCurrent().in(Amps);
    inputs.lowerMotorOutput = LowerShooter.getMotor().getStatorCurrent().in(Amps);
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
}
