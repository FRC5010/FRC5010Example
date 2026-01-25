// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

public class Launcher extends GenericSubsystem {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  /** Creates a new Launcher. */
  public Launcher() {
    super("launcher.json");

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
    io.updateSimulation();
  }

  /**
   * Run the shooter at the given speed. This is a convenience method which simply calls
   * setUpperSpeed with the given speed.
   *
   * @param speed the speed to set the upper shooter motor to, in units of RPM.
   */
  public void runShooter(double speed) {
    io.setUpperSpeed(speed);
  }

  public void setUpperSpeed(double speed) {
    io.setUpperSpeed(speed);
  }

  public void setLowerSpeed(double speed) {
    io.setLowerSpeed(speed);
  }

  public void setHoodAngle(Angle angle) {
    io.setHoodAngle(angle);
  }

  public void setTurretRotation(Angle angle) {
    io.setTurretRotation(angle);
  }

  /**
   * A command which tracks a target using the turret rotation and hood angle. The command will
   * continuously set the turret rotation and hood angle to the angle of the target relative to the
   * robot.
   *
   * @param targetSupplier a supplier which returns the target pose to track.
   * @return a command which tracks the target with the turret rotation and hood angle.
   */
  public Command trackTargetCommand(Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          Translation2d targetPose = targetSupplier.get();
          setTurretRotation(targetPose.getAngle().getMeasure());
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
        });
  }

  /**
   * Checks if the robot is at the desired speed and angle. This method returns true if the robot
   * speed and angle are within the allowed tolerances of the desired values.
   *
   * @return true if the robot is at the desired speed and angle, false otherwise.
   */
  public boolean atGoal() {
    return inputs.upperSpeedAtGoal
        && inputs.lowerSpeedAtGoal
        && inputs.hoodAngleAtGoal
        && inputs.turretAngleAtGoal;
  }
}
