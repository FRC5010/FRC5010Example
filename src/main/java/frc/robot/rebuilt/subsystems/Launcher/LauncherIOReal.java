// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import java.util.Map;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;

  public LauncherIOReal(Map<String, Object> devices) {
    this.devices = devices;
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {}
}
