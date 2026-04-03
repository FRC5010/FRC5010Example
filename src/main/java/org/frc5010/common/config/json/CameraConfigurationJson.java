// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.sensors.camera.FiducialTargetCamera;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.QuestNavInterface;
import org.frc5010.common.sensors.camera.SimulatedCamera;
import org.frc5010.common.sensors.camera.SimulatedFiducialTargetCamera;
import org.frc5010.common.sensors.camera.SimulatedVisualTargetCamera;
import org.frc5010.common.subsystems.FiducialTargetSystem;
import org.frc5010.common.subsystems.VisibleTargetSystem;
import org.frc5010.common.vision.AprilTags;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Base configuration data class for camera systems in an FRC robot.
 *
 * <p>Concrete camera types should extend this class and implement {@link
 * #configureCamera(GenericRobot)}.
 */
public abstract class CameraConfigurationJson {
  /** Constant identifier for AprilTag detection mode */
  public static String APRIL_TAG = "apriltag";
  /** Constant identifier for target tracking mode */
  public static String TARGET = "target";

  /** Unique name of the camera used as a subsystem identifier */
  public String name;
  /**
   * The use case for this camera. Valid values include:
   *
   * <ul>
   *   <li>"target" - for target tracking systems
   *   <li>"apriltag" - for AprilTag-based pose estimation
   *   <li>"quest" - for QuestNav visual odometry
   * </ul>
   */
  public String use;
  /**
   * The pose estimation strategy for PhotonVision cameras (e.g., "AVERAGE_BEST_TARGETS",
   * "LOWEST_AMBIGUITY"). For YaLL cameras, use "MEGATAG1" or "MEGATAG2". Defaults to "none" if not
   * using multi-target pose estimation.
   */
  public String strategy = "none";
  /** The SmartDashboard column index for this camera's telemetry display */
  public int column = 0;
  /** Camera pose relative to the robot center. */
  public Pose3dJson pose = new Pose3dJson();
  /** Horizontal resolution of the camera in pixels */
  public int width = 800;
  /** Vertical resolution of the camera in pixels */
  public int height = 600;
  /** Camera field of view (FOV) in degrees */
  public double fov = 70;
  /** Optional height of the target in meters (used for target tracking mode) */
  public double targetHeight = 0;
  /** Whether to view game pieces in simulation */
  public boolean viewGamePieces = true;
  /**
   * Optional array of AprilTag fiducial IDs to track. If empty, all AprilTags may be detected
   * depending on the configuration strategy.
   */
  public int[] targetFiducialIds = new int[0];

  /**
   * Configures the camera system based on the provided robot and current configuration.
   *
   * @param robot the {@link GenericRobot} instance to configure the camera for
   */
  public abstract void configureCamera(GenericRobot robot);

  /**
   * Sets whether to view game pieces in simulation.
   *
   * @param viewGamePieces whether to enable or disable viewing game pieces in simulation
   */
  public void setViewGamePieces(boolean viewGamePieces) {
    this.viewGamePieces = viewGamePieces;
  }

  /**
   * Returns whether the camera system can view game pieces in simulation mode.
   *
   * @return whether the camera system can view game pieces in simulation mode
   */
  public boolean canViewGamePieces() {
    return viewGamePieces;
  }

  protected Transform3d getRobotToCameraTransform() {
    return new Transform3d(pose.getPose3d().getTranslation(), pose.getPose3d().getRotation());
  }

  protected List<Integer> buildTargetFiducialIdList() {
    List<Integer> targetFiducialIdList = new ArrayList<>();
    for (int targetFiducialId : targetFiducialIds) {
      targetFiducialIdList.add(targetFiducialId);
    }
    return targetFiducialIdList;
  }

  protected GenericCamera createSimulatedCamera(GenericRobot robot, Transform3d robotToCamera) {
    if (!"none".equalsIgnoreCase(strategy)) {
      if (targetFiducialIds.length > 0) {
        return new SimulatedCamera(
            name,
            column,
            AprilTags.aprilTagFieldLayout,
            PoseStrategy.valueOf(strategy),
            robotToCamera,
            robot.getPoseSupplier(),
            buildTargetFiducialIdList(),
            width,
            height,
            fov);
      }
      return new SimulatedCamera(
          name,
          column,
          AprilTags.aprilTagFieldLayout,
          robotToCamera,
          robot.getSimulatedPoseSupplier(),
          width,
          height,
          fov);
    }
    if (targetFiducialIds.length > 0) {
      return new SimulatedFiducialTargetCamera(
          name,
          column,
          AprilTags.aprilTagFieldLayout,
          robotToCamera,
          robot.getSimulatedPoseSupplier(),
          buildTargetFiducialIdList(),
          width,
          height,
          fov);
    }
    if (targetHeight > 0) {
      return new SimulatedVisualTargetCamera(
          name,
          column,
          AprilTags.aprilTagFieldLayout,
          robotToCamera,
          robot.getSimulatedPoseSupplier(),
          width,
          height,
          fov);
    }
    if (!"quest".equalsIgnoreCase(use)) {
      return new SimulatedCamera(
          name,
          column,
          AprilTags.aprilTagFieldLayout,
          robotToCamera,
          robot.getSimulatedPoseSupplier(),
          width,
          height,
          fov);
    }
    return null;
  }

  protected void registerCamera(GenericRobot robot, GenericCamera camera) {
    if (camera != null) {
      camera.setCanViewGamePieces(viewGamePieces);
    }
    GenericDrivetrain drivetrain = (GenericDrivetrain) robot.getSubsystem("drivetrain");
    switch (use) {
      case "target":
        {
          if (targetFiducialIds.length > 0) {
            robot.addSubsystem(name, new FiducialTargetSystem((FiducialTargetCamera) camera));
          } else {
            robot.addSubsystem(name, new VisibleTargetSystem(camera, targetHeight));
          }
          break;
        }
      case "apriltag":
        {
          if (drivetrain != null) {
            drivetrain.getPoseEstimator().registerPoseProvider(camera);
          }
          break;
        }
      case "quest":
        {
          QuestNavInterface questNav = new QuestNavInterface(getRobotToCameraTransform());

          if (drivetrain != null) {
            questNav.withRobotSpeedSupplier(
                ((GenericSwerveDrivetrain) drivetrain)::getFieldVelocity);
            drivetrain.getPoseEstimator().registerPoseProvider(questNav);
          }
          break;
        }
      default:
        break;
    }
  }

  protected <T extends Enum<T>> Optional<T> parseEnum(Class<T> enumType, String value, T fallback) {
    if (value == null || value.isBlank()) {
      return Optional.ofNullable(fallback);
    }
    try {
      return Optional.of(Enum.valueOf(enumType, value));
    } catch (IllegalArgumentException ex) {
      System.err.println("Invalid enum value '" + value + "' for " + enumType.getSimpleName());
      return Optional.ofNullable(fallback);
    }
  }

  protected boolean isReal() {
    return RobotBase.isReal();
  }
}
