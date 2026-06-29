// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.frc5010.common.HalTestBase;
import org.junit.jupiter.api.Test;

/**
 * Verifies that a missing required config file now produces a clear {@link ConfigException} (instead
 * of a silently-disabled {@code assert}). {@code checkDirectory} runs first in the {@link
 * RobotParser} constructor, before the robot argument is used, so a null robot is fine here.
 */
class ConfigExceptionTest extends HalTestBase {

  @Test
  void robotParserThrowsConfigExceptionWhenRobotJsonMissing() {
    String missingDir = "nonexistent_robot_dir_for_test";
    ConfigException ex =
        assertThrows(ConfigException.class, () -> new RobotParser(missingDir, null));
    assertTrue(ex.getMessage().contains("robot.json"), "message should name the missing file");
    assertTrue(ex.getMessage().contains(missingDir), "message should include the directory path");
  }
}
