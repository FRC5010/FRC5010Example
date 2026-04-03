// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.YaLLCamera;

/** JSON configuration for YaLL (Limelight) cameras. */
public class YaLLCameraConfigurationJson extends CameraConfigurationJson {
  /** YaLL LED mode (e.g. PipelineControl, ForceOn, ForceOff, ForceBlink). */
  public String yallLedMode = "";
  /** YaLL stream mode (e.g. Standard, PictureInPictureMain). */
  public String yallStreamMode = "";
  /** YaLL IMU mode (e.g. InternalImu, ExternalImu). */
  public String yallImuMode = "";
  /** YaLL fiducial downscaling override (e.g. Pipeline, NoDownscale). */
  public String yallFiducialDownscaling = "";
  /** YaLL pipeline index override. */
  public int yallPipelineIndex = -1;
  /** YaLL priority AprilTag ID override. */
  public int yallPriorityTagId = -1;
  /** YaLL IMU assist alpha value; negative values disable configuration. */
  public double yallImuAssistAlpha = -1;

  @Override
  public void configureCamera(GenericRobot robot) {
    GenericCamera camera = null;
    Transform3d robotToCamera = getRobotToCameraTransform();
    if (isReal()) {
      String yallStrategy = "none".equalsIgnoreCase(strategy) ? "" : strategy;
      LimelightPoseEstimator.EstimationMode estimationMode =
          parseEnum(
                  LimelightPoseEstimator.EstimationMode.class,
                  yallStrategy,
                  LimelightPoseEstimator.EstimationMode.MEGATAG2)
              .orElse(LimelightPoseEstimator.EstimationMode.MEGATAG2);
      YaLLCamera yallCamera = new YaLLCamera(name, column, robotToCamera, estimationMode);
      applyYaLLSettings(yallCamera);
      camera = yallCamera;
    } else {
      camera = createSimulatedCamera(robot, robotToCamera);
    }
    registerCamera(robot, camera);
  }

  private void applyYaLLSettings(YaLLCamera camera) {
    LimelightSettings settings = camera.getSettings();
    boolean updated = false;

    if (!yallLedMode.isBlank()) {
      updated |=
          parseEnum(LimelightSettings.LEDMode.class, yallLedMode, null)
              .map(
                  mode -> {
                    settings.withLimelightLEDMode(mode);
                    return true;
                  })
              .orElse(false);
    }

    if (!yallStreamMode.isBlank()) {
      updated |=
          parseEnum(LimelightSettings.StreamMode.class, yallStreamMode, null)
              .map(
                  mode -> {
                    settings.withStreamMode(mode);
                    return true;
                  })
              .orElse(false);
    }

    if (!yallImuMode.isBlank()) {
      updated |=
          parseEnum(LimelightSettings.ImuMode.class, yallImuMode, null)
              .map(
                  mode -> {
                    settings.withImuMode(mode);
                    return true;
                  })
              .orElse(false);
    }

    if (!yallFiducialDownscaling.isBlank()) {
      updated |=
          parseEnum(LimelightSettings.DownscalingOverride.class, yallFiducialDownscaling, null)
              .map(
                  downscale -> {
                    settings.withFiducialDownscalingOverride(downscale);
                    return true;
                  })
              .orElse(false);
    }

    if (yallPipelineIndex >= 0) {
      settings.withPipelineIndex(yallPipelineIndex);
      updated = true;
    }

    if (yallPriorityTagId >= 0) {
      settings.withPriorityTagId(yallPriorityTagId);
      updated = true;
    }

    if (yallImuAssistAlpha >= 0) {
      settings.withImuAssistAlpha(yallImuAssistAlpha);
      updated = true;
    }

    if (targetFiducialIds.length > 0) {
      List<Integer> tagIds = buildTargetFiducialIdList();
      settings.withAprilTagIdFilter(tagIds);
      updated = true;
    }

    if (updated) {
      settings.save();
    }
  }
}
