// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.LauncherCommands.LauncherState;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;

public class Launcher extends GenericSubsystem {
  private final LauncherIO io;
  private final Arm hood;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  public static Transform3d robotToTurret = new Transform3d();

  /** Creates a new Launcher. */
  public Launcher() {
    super("launcher.json");
    Pivot turret = (Pivot) devices.get("turret");
    hood = (Arm) devices.get("hood");
    robotToTurret = new Transform3d(turret.getRelativeMechanismPosition(), new Rotation3d());

    if (RobotBase.isSimulation()) {
      io = new LauncherIOSim(devices);
    } else {
      io = new LauncherIOReal(devices);
    }
  }

  /**
   * Updates the inputs of the Launcher subsystem from the physical devices.
   *
   * <p>This method is called periodically by the GenericSubsystem class.
   */
  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }

  /**
   * Called every time the scheduler runs while the robot is in simulation mode. Used to update
   * simulation models.
   */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    io.updateSimulation(this, Rebuilt.indexer);
  }

  /**
   * Run the shooter at the given speed. This is a convenience method which simply calls
   * setUpperSpeed with the given speed.
   *
   * @param speed the speed to set the upper shooter motor to, in units of RPM.
   */
  public void runShooter(double speed) {
    io.runShooter(speed);
  }

  public void setHoodAngle(Angle angle) {
    io.setHoodAngle(angle);
  }

  public void setTurretRotation(Angle angle) {
    io.setTurretRotation(angle);
  }

  public boolean isShooting() {
    return inputs.stateCurrent == LauncherState.PREP || inputs.stateCurrent == LauncherState.PRESET;
  }

  public Command getHoodSysIdCommand() {
    return io.getHoodSysIdCommand();
  }

  public Command getTurretSysIdCommand() {
    return io.getTurretSysIdCommand(this);
  }

  public Command getFlyWheelSysIdCommand() {
    return io.getFlyWheelSysIdCommand(this);
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.runOnce(
        () -> {
          io.stopAllMotors();
        },
        this);
  }

  public void stopAllMotors() {
    io.stopAllMotors();
  }

  public Command trackTargetCommand() {
    return Commands.run(
        () -> {
          io.setHoodAngle(inputs.hoodAngleCalculated);
          io.setTurretRotation(inputs.turretAngleCalculated);
          io.setFlyWheelVelocity(inputs.flyWheelSpeedCalculated);
        });
  }

  public Command trackTargetCommand(double speed) {
    return Commands.run(
        () -> {
          io.setHoodAngle(hood.getMotorController().getConfig().getMechanismLowerLimit().get());
          io.setTurretRotation(inputs.turretAngleCalculated);
          io.setFlyWheelVelocity(RPM.of(speed));
        });
  }

  /**
   * A command which stops the tracking of a target and resets the turret rotation and hood angle to
   * 0 degrees.
   *
   * @return a command which stops tracking and resets the turret rotation and hood angle.
   */
  public Command stopTrackingCommand() {
    return Commands.runOnce(
        () -> {
          setTurretRotation(Degrees.of(0));
          setHoodAngle(Degrees.of(0));
          runShooter(0);
        });
  }

  /**
   * Checks if the robot is at the desired speed and angle. This method returns true if the robot
   * speed and angle are within the allowed tolerances of the desired values.
   *
   * @return true if the robot is at the desired speed and angle, false otherwise.
   */
  public boolean isAtGoal() {
    return inputs.flyWheelSpeedAtGoal && inputs.hoodAngleAtGoal && inputs.turretAngleAtGoal;
  }

  public boolean isRequested(LauncherState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(LauncherState state) {
    return inputs.stateCurrent == state;
  }

  public void setCurrentState(LauncherState state) {
    inputs.stateCurrent = state;
  }

  public void setRequestedState(LauncherState state) {
    inputs.stateRequested = state;
  }

  public LauncherState getCurrentState() {
    return inputs.stateCurrent;
  }

  public void usePresets(Angle hoodAngle, Angle turretAngle, AngularVelocity flywheelSpeed) {
    io.setHoodAngle(hoodAngle);
    io.setTurretRotation(turretAngle);
    io.setFlyWheelVelocity(flywheelSpeed);
  }
}
