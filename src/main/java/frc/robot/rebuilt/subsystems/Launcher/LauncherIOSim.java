// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.intake.IntakeIOSim;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Add your docs here. */
public class LauncherIOSim extends LauncherIOReal {
  protected GamePieceProjectile gamePieceProjectile;
  protected Map<String, Object> devices;

  public LauncherIOSim(Map<String, Object> devices) {
    super(devices);
  }

  @Override
  public void updateSimulation(Launcher launcher) {
    int amount = IntakeIOSim.intakeSimulation.getGamePiecesAmount();
    // Update simulated mechanism states here
    // We should simulate a shot rate of about 10-15 gamepieces per second
    // Every other time this is called, determine a randome number and if > 0.5, shoot a gamepiece.
    // This would mean we try to shoot 25 times per second, and on average shoot about 12-13
    // gamepieces per second.
    if (Math.random() > 0.5 && amount > 0) {
      if (launcher.isAtGoal() && IntakeIOSim.intakeSimulation.obtainGamePieceFromIntake()) {
        Pose2d worldPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
        gamePieceProjectile =
            new RebuiltFuelOnFly(
                    worldPose.getTranslation(),
                    flyWheel.getRelativeMechanismPosition().toTranslation2d(),
                    Rebuilt.drivetrain.getFieldVelocity(),
                    worldPose.getRotation(),
                    flyWheel.getRelativeMechanismPosition().getMeasureZ(),
                    getFlyWheelExitSpeed(flyWheel.getSpeed()),
                    Degrees.of(90.0).minus(hood.getAngle()))
                .withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) -> {
                      Logger.recordOutput(
                          "Launcher/GamePieceTrajectory", pose3ds.toArray(Pose3d[]::new));
                    });
        SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
        // Create a new gamepiece on-the-fly and add it to the field simulation
      }
    }
  }
}
