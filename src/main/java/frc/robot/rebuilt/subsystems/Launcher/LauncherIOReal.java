// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.commands.LauncherCommands;
import java.util.Map;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  private Pivot Turret;
  private Arm Hood;
  private FlyWheel flyWheel;

  public LauncherIOReal(Map<String, Object> devices) {
    this.devices = devices;
    Turret = (Pivot) devices.get("turret");
    Hood = (Arm) devices.get("hood");
    flyWheel = (FlyWheel) devices.get("flywheel");
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.flyWheelSpeedDesired =
        flyWheel
            .getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it.in(RPM))
            .orElse(0.0);
    inputs.hoodAngleDesired =
        Hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    inputs.turretAngleDesired =
        Turret.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));

    inputs.flyWheelSpeedActual = flyWheel.getSpeed().in(RPM);
    inputs.hoodAngleActual = Hood.getAngle();
    inputs.turretAngleActual = Turret.getAngle();

    inputs.flyWheelSpeedError = inputs.flyWheelSpeedActual - inputs.flyWheelSpeedDesired;
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.flyWheelSpeedError) <= Constants.LauncherConstants.SHOOTER_TOLERANCE_RPM;
    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.LauncherConstants.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError)
            <= Constants.LauncherConstants.TURRET_ANGLE_TOLERANCE_DEGREES;

    inputs.hoodVelocity = Hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.turretVelocity =
        Turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.flyWheelMotorOutput = flyWheel.getMotor().getStatorCurrent().in(Amps);

    inputs.robotToTarget = LauncherCommands.getRobotToTarget();

    inputs.targetDistance = Meters.of(inputs.robotToTarget.getDistance(new Translation2d()));
  }

  public void runShooter(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }

  public void setUpperSpeed(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }

  public void setLowerSpeed(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }

  public void setHoodAngle(Angle angle) {
    Hood.getMotorController().setPosition(angle);
  }

  public void setTurretRotation(Angle angle) {
    Turret.getMotorController().setPosition(angle);
  }

  public void trackTarget() {} // Placeholder
}
