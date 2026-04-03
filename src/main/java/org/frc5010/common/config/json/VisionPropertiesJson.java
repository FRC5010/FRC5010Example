// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.vision.AprilTags;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/** JSON class with an array of cameras to configure */
public class VisionPropertiesJson {
  /** List of camera entries that define camera type and config file. */
  public List<CameraEntry> cameras = new ArrayList<>();

  public String aprilTagLayout = "default";
  public String simulatedField = "default";
  public Map<String, String[]> gamePieces = new HashMap<>();
  public boolean viewGamePieces = true;

  /**
   * Creates cameras for a given robot using the provided map of camera configurations.
   *
   * @param robot the robot to add the cameras to
   * @param map a map of camera configurations, where the key is the camera name and the value is
   *     the configuration object
   */
  public void createCameraSystem(GenericRobot robot, Map<String, CameraConfigurationJson> map) {
    map.keySet()
        .forEach(
            it -> {
              map.get(it).configureCamera(robot);
            });
  }

  /**
   * Reads in cameras from the provided directory.
   *
   * @param directory the directory to read from
   * @return the map of camera configurations
   */
  public Map<String, CameraConfigurationJson> readCameraSystem(File directory) throws IOException {
    if (!simulatedField.equalsIgnoreCase("default")) {
      try {
        SimulatedArena arena =
            Class.forName(simulatedField)
                .asSubclass(SimulatedArena.class)
                .getDeclaredConstructor()
                .newInstance();
        if (!gamePieces.isEmpty()) {
          arena.clearGamePieces();
          for (String key : gamePieces.keySet()) {
            arena.addGamePiece(
                Class.forName(gamePieces.get(key)[0])
                    .asSubclass(GamePieceOnFieldSimulation.class)
                    .getDeclaredConstructor()
                    .newInstance());
          }
        }
        SimulatedArena.overrideInstance(arena);
      } catch (ClassNotFoundException
          | NoSuchMethodException
          | InstantiationException
          | IllegalAccessException
          | InvocationTargetException e) {
        System.err.println("Error creating arena instance: " + e.getMessage());
        e.printStackTrace();
        throw new RuntimeException(e);
      }
    }
    switch (aprilTagLayout) {
      case "5010" -> AprilTags.setAprilTagFieldLayout(AprilTags.aprilTagRoomLayout);
      case "default" -> {
        AprilTagFieldLayout layout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        AprilTags.setAprilTagFieldLayout(layout);
      }
      default -> {
        AprilTagFieldLayout layout;
        try {
          layout =
              AprilTagFieldLayout.loadFromResource(
                  AprilTagFields.valueOf(aprilTagLayout).m_resourceFile);
          AprilTags.setAprilTagFieldLayout(layout);
        } catch (IllegalArgumentException e) {
          layout = AprilTagFieldLayout.loadFromResource(aprilTagLayout);
          AprilTags.setAprilTagFieldLayout(layout);
        }
      }
    }

    Map<String, CameraConfigurationJson> camerasMap = new HashMap<>();
    for (CameraEntry entry : cameras) {
      String configFile = entry.resolveConfigFile();
      File cameraFile = new File(directory, "cameras/" + configFile);
      assert cameraFile.exists();
      CameraConfigurationJson camera = readCameraConfig(cameraFile, entry.type);
      if (camera.name == null || camera.name.isBlank()) {
        camera.name = entry.name;
      }
      camera.setViewGamePieces(viewGamePieces);
      camerasMap.put(camera.name, camera);
    }
    return camerasMap;
  }

  private CameraConfigurationJson readCameraConfig(File cameraFile, String type)
      throws IOException {
    ObjectMapper mapper = new ObjectMapper();
    if (type == null) {
      throw new IllegalArgumentException("Camera type cannot be null");
    }
    return switch (type.toLowerCase()) {
      case "limelight", "yall" -> mapper.readValue(cameraFile, YaLLCameraConfigurationJson.class);
      case "photonvision" -> mapper.readValue(
          cameraFile, PhotonVisionCameraConfigurationJson.class);
      case "quest" -> mapper.readValue(cameraFile, QuestCameraConfigurationJson.class);
      default -> throw new IllegalArgumentException("Unsupported camera type: " + type);
    };
  }

  /** Entry defining the type and config file of a camera. */
  public static class CameraEntry {
    /** Camera name used for registration. */
    public String name;
    /** Camera type, e.g. limelight, photonvision, yall, quest. */
    public String type;
    /** Optional config filename. Defaults to name + ".json" when unset. */
    public String config;

    public String resolveConfigFile() {
      if (config != null && !config.isBlank()) {
        return config;
      }
      return name + ".json";
    }
  }
}
