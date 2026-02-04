// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.commands.LauncherCommands;
import java.util.Map;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  private Pivot turret;
  protected Arm hood;
  protected FlyWheel flyWheel;

  public LauncherIOReal(Map<String, Object> devices) {
    this.devices = devices;
    turret = (Pivot) devices.get("turret");
    hood = (Arm) devices.get("hood");
    flyWheel = (FlyWheel) devices.get("flywheel");
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    ShotCalculator.ShootingParameters params = ShotCalculator.getInstance().getParameters();
    if (params != null && params.isValid()) {
      inputs.hoodAngleCalculated = Degrees.of(params.hoodAngle());
      inputs.turretAngleCalculated = params.turretAngle().getMeasure();
      inputs.flyWheelSpeedCalculated = RotationsPerSecond.of(params.flywheelSpeed());
      // TODO: Use parameters to set turret, hood and flywheel desired values
    }

    inputs.flyWheelSpeedDesired =
        flyWheel
            .getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it)
            .orElse(RPM.of(0.0));
    inputs.hoodAngleDesired =
        hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    inputs.turretAngleDesired =
        turret.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));

    inputs.flyWheelSpeedActual = flyWheel.getSpeed();
    inputs.hoodAngleActual = hood.getAngle();
    inputs.turretAngleActual = turret.getAngle();

    inputs.flyWheelSpeedError = inputs.flyWheelSpeedActual.minus(inputs.flyWheelSpeedDesired);
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.flyWheelSpeedError.in(RPM))
            <= Constants.LauncherConstants.SHOOTER_TOLERANCE_RPM;
    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.LauncherConstants.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError)
            <= Constants.LauncherConstants.TURRET_ANGLE_TOLERANCE_DEGREES;

    inputs.hoodVelocity = hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.turretVelocity =
        turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.flyWheelMotorOutput = flyWheel.getMotor().getStatorCurrent().in(Amps);

    inputs.robotToTarget = LauncherCommands.getRobotToTarget();

    inputs.targetDistance = Meters.of(inputs.robotToTarget.getDistance(new Translation2d()));
  }

  public void runShooter(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }

  public void setFlyWheelVelocity(AngularVelocity speed) {
    flyWheel.getMotor().setVelocity(speed);
  }

  public void setHoodAngle(Angle angle) {
    hood.getMotorController().setPosition(angle);
  }

  public void setTurretRotation(Angle angle) {
    turret.getMotorController().setPosition(angle);
  }

  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity) {
    return MetersPerSecond.of(
        flyWheel.getShooterConfig().getLength().get().magnitude() * (velocity.magnitude()));
  }
}
