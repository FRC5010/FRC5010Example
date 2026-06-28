// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.utils.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

/** Pure-logic tests for {@link GeomUtil} geometry conversions. No HAL dependency. */
class GeomUtilTest {
  private static final double EPS = 1e-9;

  @Test
  void toTransform2dFromTranslationHasZeroRotation() {
    Transform2d t = GeomUtil.toTransform2d(new Translation2d(1.0, 2.0));
    assertEquals(1.0, t.getX(), EPS);
    assertEquals(2.0, t.getY(), EPS);
    assertEquals(0.0, t.getRotation().getRadians(), EPS);
  }

  @Test
  void toTransform2dFromCoordinates() {
    Transform2d t = GeomUtil.toTransform2d(3.0, 4.0);
    assertEquals(3.0, t.getX(), EPS);
    assertEquals(4.0, t.getY(), EPS);
  }

  @Test
  void toTransform2dFromRotationHasZeroTranslation() {
    Transform2d t = GeomUtil.toTransform2d(Rotation2d.fromDegrees(90));
    assertEquals(0.0, t.getX(), EPS);
    assertEquals(0.0, t.getY(), EPS);
    assertEquals(Math.PI / 2, t.getRotation().getRadians(), EPS);
  }

  @Test
  void poseTransformRoundTrip() {
    Pose2d pose = new Pose2d(1.5, -2.5, Rotation2d.fromDegrees(30));
    Pose2d roundTripped = GeomUtil.toPose2d(GeomUtil.toTransform2d(pose));
    assertEquals(pose, roundTripped);
  }

  @Test
  void inverseOfPureTranslation() {
    Pose2d inv = GeomUtil.inverse(new Pose2d(1.0, 0.0, Rotation2d.kZero));
    assertEquals(-1.0, inv.getX(), EPS);
    assertEquals(0.0, inv.getY(), EPS);
    assertEquals(0.0, inv.getRotation().getRadians(), EPS);
  }

  @Test
  void inverseOfTranslationAndRotation() {
    // inverse(Pose(1,2,90deg)): rotInv = -90deg; translation (-1,-2) rotated by -90deg -> (-2, 1)
    Pose2d inv = GeomUtil.inverse(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90)));
    assertEquals(-2.0, inv.getX(), EPS);
    assertEquals(1.0, inv.getY(), EPS);
    assertEquals(-Math.PI / 2, inv.getRotation().getRadians(), EPS);
  }

  @Test
  void inverseComposedWithOriginalIsIdentity() {
    Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(40));
    Transform2d composed =
        GeomUtil.toTransform2d(pose).plus(GeomUtil.toTransform2d(GeomUtil.inverse(pose)));
    assertEquals(0.0, composed.getX(), EPS);
    assertEquals(0.0, composed.getY(), EPS);
    assertEquals(0.0, composed.getRotation().getRadians(), EPS);
  }

  @Test
  void multiplyScalesAllTwistComponents() {
    Twist2d scaled = GeomUtil.multiply(new Twist2d(1.0, 2.0, 3.0), 2.0);
    assertEquals(2.0, scaled.dx, EPS);
    assertEquals(4.0, scaled.dy, EPS);
    assertEquals(6.0, scaled.dtheta, EPS);
  }

  @Test
  void toTwist2dMapsChassisSpeeds() {
    Twist2d twist = GeomUtil.toTwist2d(new ChassisSpeeds(1.0, 2.0, 3.0));
    assertEquals(1.0, twist.dx, EPS);
    assertEquals(2.0, twist.dy, EPS);
    assertEquals(3.0, twist.dtheta, EPS);
  }

  @Test
  void toTransform2dFromTransform3dDropsZ() {
    Transform3d t3 =
        new Transform3d(new Translation3d(1.0, 2.0, 9.0), new edu.wpi.first.math.geometry.Rotation3d());
    Transform2d t2 = GeomUtil.toTransform2d(t3);
    assertEquals(1.0, t2.getX(), EPS);
    assertEquals(2.0, t2.getY(), EPS);
  }

  @Test
  void withTranslationAndWithRotation() {
    Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(10));
    Pose2d movedT = GeomUtil.withTranslation(pose, new Translation2d(5.0, 6.0));
    assertEquals(5.0, movedT.getX(), EPS);
    assertEquals(6.0, movedT.getY(), EPS);
    assertEquals(Rotation2d.fromDegrees(10).getRadians(), movedT.getRotation().getRadians(), EPS);

    Pose2d movedR = GeomUtil.withRotation(pose, Rotation2d.fromDegrees(90));
    assertEquals(1.0, movedR.getX(), EPS);
    assertEquals(1.0, movedR.getY(), EPS);
    assertEquals(Math.PI / 2, movedR.getRotation().getRadians(), EPS);
  }
}
