// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.PhotonVisionCamera;
import org.frc5010.common.sensors.camera.PhotonVisionFiducialTargetCamera;
import org.frc5010.common.sensors.camera.PhotonVisionPoseCamera;
import org.frc5010.common.sensors.camera.PhotonVisionVisualTargetCamera;
import org.frc5010.common.vision.AprilTags;

/** JSON configuration for PhotonVision cameras. */
public class PhotonVisionCameraConfigurationJson extends CameraConfigurationJson {

  @Override
  public void configureCamera(GenericRobot robot) {
    GenericCamera camera = null;
    Transform3d robotToCamera = getRobotToCameraTransform();
    if (isReal()) {
      List<Integer> targetFiducialIdList = buildTargetFiducialIdList();
      if (!"none".equalsIgnoreCase(strategy)) {
        if (!targetFiducialIdList.isEmpty()) {
          camera =
              new PhotonVisionPoseCamera(
                  name,
                  column,
                  AprilTags.aprilTagFieldLayout,
                  robotToCamera,
                  robot.getPoseSupplier(),
                  targetFiducialIdList);
        } else {
          camera =
              new PhotonVisionPoseCamera(
                  name,
                  column,
                  AprilTags.aprilTagFieldLayout,
                  robotToCamera,
                  robot.getPoseSupplier());
        }
      } else if (!targetFiducialIdList.isEmpty()) {
        camera =
            new PhotonVisionFiducialTargetCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                robotToCamera,
                robot.getPoseSupplier(),
                targetFiducialIdList);
      } else if (targetHeight > 0) {
        camera = new PhotonVisionVisualTargetCamera(name, column, robotToCamera);
      } else {
        camera = new PhotonVisionCamera(name, column, robotToCamera);
      }
    } else {
      camera = createSimulatedCamera(robot, robotToCamera);
    }
    registerCamera(robot, camera);
  }
}
