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
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.config.json.AKitSwerveDrivetrainJson;
import org.frc5010.common.config.json.CameraConfigurationJson;
import org.frc5010.common.config.json.DriveteamControllerAxisJson;
import org.frc5010.common.config.json.DriveteamControllerJson;
import org.frc5010.common.config.json.DriveteamControllersJson;
import org.frc5010.common.config.json.RobotJson;
import org.frc5010.common.config.json.RobotsJson;
import org.frc5010.common.config.json.UserModeJson;
import org.frc5010.common.config.json.VisionPropertiesJson;
import org.frc5010.common.config.json.YAGSLDriveModuleJson;
import org.frc5010.common.config.json.YAGSLDrivetrainJson;
import org.junit.jupiter.api.Test;

/**
 * Deserializes the real deploy configuration under {@code src/main/deploy/} into its Jackson POJOs.
 *
 * <p>Uses a bare {@link ObjectMapper} exactly as the production parsers do (which by default fails
 * on unknown properties), so these tests are a guard against schema&lt;-&gt;POJO drift: if a deploy
 * file gains a field the POJO does not model (or vice-versa), the corresponding test fails. Pure
 * JSON parsing — no HAL/NetworkTables.
 */
class ConfigDeserializationTest {
  /** Gradle runs tests with the project directory as the working directory. */
  private static final File DEPLOY = new File("src/main/deploy");

  private static final String[] ROBOT_DIRS = {"baby_swerve", "basic_robot"};

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private static <T> T read(File file, Class<T> type) {
    assertTrue(file.exists(), "Missing deploy file: " + file.getPath());
    return assertDoesNotThrow(
        () -> MAPPER.readValue(file, type),
        "Failed to deserialize " + file.getPath() + " into " + type.getSimpleName());
  }

  private static List<File> jsonFilesIn(File dir) {
    List<File> files = new ArrayList<>();
    File[] listing = dir.listFiles((d, name) -> name.endsWith(".json"));
    if (listing != null) {
      for (File f : listing) {
        files.add(f);
      }
    }
    return files;
  }

  @Test
  void robotsJsonDeserializes() {
    RobotsJson robots = read(new File(DEPLOY, "robots.json"), RobotsJson.class);
    assertNotNull(robots.robots);
    assertFalse(robots.robots.isEmpty(), "robots.json should declare at least one robot");
  }

  @Test
  void perRobotConfigDeserializes() {
    for (String dir : ROBOT_DIRS) {
      File robotDir = new File(DEPLOY, dir);

      RobotJson robot = read(new File(robotDir, "robot.json"), RobotJson.class);
      assertNotNull(robot.driveType, dir + ": driveType");
      assertFalse(robot.driveType.isBlank(), dir + ": driveType blank");

      read(new File(robotDir, "cameras.json"), VisionPropertiesJson.class);
      read(new File(robotDir, "controllers.json"), DriveteamControllersJson.class);

      UserModeJson comp = read(new File(robotDir, "competition_mode.json"), UserModeJson.class);
      assertTrue(comp.maxSpeed > 0, dir + ": competition maxSpeed");
      read(new File(robotDir, "demo_mode.json"), UserModeJson.class);
    }
  }

  @Test
  void controllerAndAxisConfigsDeserialize() {
    for (String dir : ROBOT_DIRS) {
      File controllers = new File(DEPLOY, dir + "/controllers");
      for (File f : jsonFilesIn(controllers)) {
        DriveteamControllerJson c = read(f, DriveteamControllerJson.class);
        assertNotNull(c.name, f.getName() + ": name");
      }
      for (File f : jsonFilesIn(new File(controllers, "axis"))) {
        read(f, DriveteamControllerAxisJson.class);
      }
    }
  }

  @Test
  void cameraConfigsDeserialize() {
    for (String dir : ROBOT_DIRS) {
      for (File f : jsonFilesIn(new File(DEPLOY, dir + "/cameras"))) {
        CameraConfigurationJson cam = read(f, CameraConfigurationJson.class);
        assertNotNull(cam.name, f.getName() + ": name");
      }
    }
  }

  @Test
  void driveModuleConfigsDeserialize() {
    for (String dir : ROBOT_DIRS) {
      for (File f : jsonFilesIn(new File(DEPLOY, dir + "/drive_modules"))) {
        read(f, YAGSLDriveModuleJson.class);
      }
    }
  }

  @Test
  void drivetrainConfigsDeserialize() {
    for (String dir : ROBOT_DIRS) {
      read(new File(DEPLOY, dir + "/yagsl_drivetrain.json"), YAGSLDrivetrainJson.class);
    }
    read(
        new File(DEPLOY, "basic_robot/akit_swerve_drivetrain.json"),
        AKitSwerveDrivetrainJson.class);
  }
}
