// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.commands.LauncherCommands;
import java.util.Map;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.simulation.Sensor;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  private Pivot turret;
  protected Arm hood;
  protected FlyWheel flyWheel;
  protected CANcoder crtEncoder40;
  protected CANcoder crtEncoder36;
  protected final Sensor crtSensor40;
  protected final Sensor crtSensor36;
  protected EasyCRT easyCrtSolver;

  public LauncherIOReal(Map<String, Object> devices) {
    this.devices = devices;
    turret = (Pivot) devices.get("turret");
    hood = (Arm) devices.get("hood");
    flyWheel = (FlyWheel) devices.get("flywheel");
    crtEncoder40 = new CANcoder(1, "canivore");
    crtEncoder36 = new CANcoder(2, "canivore");
    double sensor40Sim = 10;
    double sensor36Sim = 20;
    crtSensor40 =
        new SensorConfig("CRT sensor 40")
            .withField("angle", () -> crtEncoder40.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor40Sim)
            .getSensor();
    crtSensor36 =
        new SensorConfig("CRT sensor 36")
            .withField("angle", () -> crtEncoder36.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor36Sim)
            .getSensor();

    EasyCRTConfig easyCrt =
        new EasyCRTConfig(
                () -> Degrees.of(crtSensor40.getAsDouble("angle")),
                () -> Degrees.of(crtSensor36.getAsDouble("angle")))
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 30.0,
                /* driveGearTeeth */ 12,
                /* encoder1Pinion */ 40,
                /* encoder2Pinion */ 36)
            .withAbsoluteEncoderOffsets(
                Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Degrees.of(-165), Degrees.of(165)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(false, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    easyCrtSolver = new EasyCRT(easyCrt);
    // // Test Values
    // SmartDashboard.putNumber(
    //     "Unique Coverage", easyCrt.getUniqueCoverage().orElse(Degrees.of(0.0)).in(Degrees));
    // SmartDashboard.putBoolean("Coverage Satisfies Range", easyCrt.coverageSatisfiesRange());

    turret.setAngle(easyCrtSolver.getAngleOptional().orElse(Degrees.of(0.0)));
  }

  @Override()
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
        flyWheel.getShooterConfig().getCircumference().in(Meters)
            * (velocity.in(RotationsPerSecond)));
  }

  public Command getHoodSysIdCommand() {
    return hood.sysId(Volts.of(12), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  public Command getTurretSysIdCommand() {
    return turret.sysId(Volts.of(12), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  public Command getFlyWheelSysIdCommand() {
    return flyWheel.sysId(Volts.of(12), Volts.of(1).per(Seconds), Seconds.of(5));
  }
}
