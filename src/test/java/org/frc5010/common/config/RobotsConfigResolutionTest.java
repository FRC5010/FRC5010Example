// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.json.RobotsJson;
import org.frc5010.common.utils.RobotIdentity;
import org.junit.jupiter.api.Test;

/**
 * Validates that every robot declared in {@code robots.json} resolves to a constructible {@link
 * GenericRobot} subclass, mirroring the reflection that {@link RobotsJson#createRobot()} performs at
 * runtime ({@code Class.forName(...).asSubclass(GenericRobot.class).getDeclaredConstructor(String.class)}).
 *
 * <p>This guards against drift between {@code robots.json} and the Java robot classes — a renamed or
 * moved class, a typo in {@code robotClass}, or a removed {@code (String directory)} constructor
 * would all fail here.
 *
 * <p>Note: this deliberately stops short of <em>instantiating</em> the robots. Full construction
 * currently throws under the test JVM because optional-config parsers (e.g. {@code LEDStripParser})
 * use Java {@code assert} for file-existence checks, which is a no-op on the robot (assertions off)
 * but fires with {@code -ea} enabled in tests. Once that assert-based config handling is replaced
 * (Phase 2), this can be upgraded to assert each robot builds a {@code DRIVETRAIN} subsystem.
 */
class RobotsConfigResolutionTest {

  @Test
  void everyRobotClassResolvesToConstructibleGenericRobot() throws Exception {
    RobotsJson robots =
        new ObjectMapper().readValue(new File("src/main/deploy/robots.json"), RobotsJson.class);
    assertNotNull(robots.robots);
    assertFalse(robots.robots.isEmpty(), "robots.json should declare at least one robot");

    for (Map.Entry<String, RobotIdentity> entry : robots.robots.entrySet()) {
      String robotName = entry.getKey();
      RobotIdentity identity = entry.getValue();
      assertNotNull(identity.robotClass, robotName + ": robotClass must be set");

      Class<?> clazz =
          assertDoesNotThrow(
              () -> Class.forName(identity.robotClass, false, getClass().getClassLoader()),
              robotName + ": robotClass '" + identity.robotClass + "' not found");

      assertTrue(
          GenericRobot.class.isAssignableFrom(clazz),
          robotName + ": " + identity.robotClass + " must extend GenericRobot");

      assertDoesNotThrow(
          () -> clazz.asSubclass(GenericRobot.class).getDeclaredConstructor(String.class),
          robotName + ": " + identity.robotClass + " must have a (String directory) constructor");
    }
  }
}
