// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import org.frc5010.common.arch.GenericRobot;

/** JSON configuration for Quest cameras (QuestNav). */
public class QuestCameraConfigurationJson extends CameraConfigurationJson {

  @Override
  public void configureCamera(GenericRobot robot) {
    // QuestNav registers as a pose provider without a physical camera instance.
    registerCamera(robot, null);
  }
}
