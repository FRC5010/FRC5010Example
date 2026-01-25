// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import frc.robot.rebuilt.subsystems.intake.IntakeIOSim;
import java.util.Map;

/** Add your docs here. */
public class LauncherIOSim extends LauncherIOReal {

  protected Map<String, Object> devices;

  public LauncherIOSim(Map<String, Object> devices) {
    super(devices);
  }

  @Override
  public void updateSimulation() {
    int amount = IntakeIOSim.intakeSimulation.getGamePiecesAmount();
    // Update simulated mechanism states here
    // We should simulate a shot rate of about 10-15 gamepieces per second
    // Every other time this is called, determine a randome number and if > 0.5, shoot a gamepiece.
    // This would mean we try to shoot 25 times per second, and on average shoot about 12-13
    // gamepieces per second.
    if (Math.random() > 0.5 && amount > 0) {
      if (IntakeIOSim.intakeSimulation.obtainGamePieceFromIntake()) {
        // Create a new gamepiece on-the-fly and add it to the field simulation
      }
    }
  }
}
