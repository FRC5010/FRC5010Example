// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.example.ExampleRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.HalTestBase;
import org.junit.jupiter.api.Test;

/**
 * End-to-end construction test: builds the example robot through the full JSON config path
 * ({@code basic_robot} → {@link ExampleRobot}, AKIT_SWERVE_DRIVE) and asserts it ends up with a
 * drivetrain. This became possible after Phase 2 removed the always-false {@code
 * DeviceConfigReader.checkDirectory} assert that previously aborted construction under the test
 * JVM's {@code -ea}.
 *
 * <p>Only one robot is constructed here on purpose. Building a second robot in the same JVM is
 * currently flaky: drivetrains keep shared <em>static</em> state ({@code
 * YAGSLSwerveDrivetrain.swerveDrive}, {@code SwerveDriveFunctions} sim fields, the {@code
 * OdometryThread} singleton), so the first construction leaks into the second. Once Phase 3
 * de-static-ifies that state, a {@code baby_swerve} (YAGSL) construction case can be added.
 */
class RobotConstructionTest extends HalTestBase {

  @Test
  void exampleRobotBuildsDrivetrainFromAkitConfig() {
    ExampleRobot robot = new ExampleRobot("basic_robot");
    assertNotNull(
        robot.getSubsystem(ConfigConstants.DRIVETRAIN),
        "basic_robot (AKIT) should build a drivetrain subsystem from config");
  }
}
