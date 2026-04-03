// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Arrays;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightData;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightTargetData;
import limelight.networktables.PoseEstimate;

/** A camera using the Yet Another Limelight Library (YaLL) API. */
public class YaLLCamera extends GenericCamera {
  /** The Limelight camera wrapper. */
  protected final Limelight limelight;
  /** Limelight network table data wrapper. */
  protected final LimelightData data;
  /** Limelight target data wrapper. */
  protected final LimelightTargetData targetData;
  /** Limelight pose estimator wrapper. */
  protected final LimelightPoseEstimator poseEstimator;
  /** Cached availability of this limelight on NetworkTables. */
  protected final boolean limelightAvailable;
  /** Limelight pose estimation mode. */
  protected final LimelightPoseEstimator.EstimationMode estimationMode;

  /**
   * Constructor.
   *
   * @param name the Limelight name
   * @param colIndex the Shuffleboard column index
   * @param robotToCamera the robot-to-camera transform
   * @param estimationMode the YaLL pose estimation mode (MegaTag1 or MegaTag2)
   */
  public YaLLCamera(
      String name,
      int colIndex,
      Transform3d robotToCamera,
      LimelightPoseEstimator.EstimationMode estimationMode) {
    super(name, colIndex, robotToCamera);
    this.estimationMode = estimationMode;
    limelight = new Limelight(name);
    data = limelight.getData();
    targetData = data.targetData;
    poseEstimator = limelight.createPoseEstimator(estimationMode);
    limelightAvailable = Limelight.isAvailable(name);
    applyCameraOffset();
  }

  /**
   * Constructor using MegaTag2 pose estimation.
   *
   * @param name the Limelight name
   * @param colIndex the Shuffleboard column index
   * @param robotToCamera the robot-to-camera transform
   */
  public YaLLCamera(String name, int colIndex, Transform3d robotToCamera) {
    this(name, colIndex, robotToCamera, LimelightPoseEstimator.EstimationMode.MEGATAG2);
  }

  @Override
  public void updateCameraInfo() {
    Optional<LimelightResults> results = limelight.getLatestResults();
    input.connected = limelightAvailable;
    input.hasTarget = targetData.getTargetStatus();
    input.latestTargetRotation =
        new TargetRotation(
            new Rotation3d(0.0, targetData.getVerticalOffset(), targetData.getHorizontalOffset()));
    input.latestTargetPose = input.hasTarget ? targetData.getTargetToRobot() : new Pose3d();
    input.captureTime =
        results.map(r -> (r.latency_capture + r.latency_pipeline) / 1000.0).orElse(0.0);

    Optional<PoseEstimate> poseEstimate = poseEstimator.getPoseEstimate();
    if (poseEstimate.isPresent() && poseEstimate.get().hasData) {
      PoseEstimate estimate = poseEstimate.get();
      PoseObservation observation =
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.pose,
              estimate.getAvgTagAmbiguity(),
              estimate.tagCount,
              estimate.avgTagDist,
              estimate.isMegaTag2 ? PoseObservationType.MEGATAG_2 : PoseObservationType.MEGATAG_1,
              ProviderType.FIELD_BASED);
      input.poseObservations = new PoseObservation[] {observation};
      input.tagIds =
          estimate.rawFiducials == null
              ? new int[0]
              : Arrays.stream(estimate.rawFiducials).mapToInt(fiducial -> fiducial.id).toArray();
    } else {
      input.poseObservations = new PoseObservation[0];
      input.tagIds = new int[0];
    }
  }

  @Override
  public double getTargetArea() {
    return targetData.getTargetArea();
  }

  @Override
  public ProviderType getType() {
    return ProviderType.FIELD_BASED;
  }

  /**
   * Returns a mutable settings object for configuring the Limelight.
   *
   * @return Limelight settings handle
   */
  public LimelightSettings getSettings() {
    return limelight.getSettings();
  }

  private void applyCameraOffset() {
    limelight
        .getSettings()
        .withCameraOffset(new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()))
        .save();
  }
}
