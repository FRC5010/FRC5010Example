// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.utils.geometry;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5010.common.HalTestBase;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link AllianceFlipUtil}. Uses the WPILib DriverStation sim (via {@link HalTestBase}) to
 * drive the alliance, so the flip logic is exercised for real red/blue/unknown states.
 */
class AllianceFlipUtilTest extends HalTestBase {
  private static final double FIELD_LENGTH = 16.0;
  private static final double FIELD_WIDTH = 8.0;
  private static final double EPS = 1e-9;

  @BeforeEach
  void configureField() {
    AllianceFlipUtil.configure(Meters.of(FIELD_WIDTH), Meters.of(FIELD_LENGTH));
  }

  @Test
  void blueAllianceDoesNotFlip() {
    setAllianceStation(AllianceStationID.Blue1);
    assertFalse(AllianceFlipUtil.shouldFlip());
    assertEquals(3.0, AllianceFlipUtil.applyX(3.0), EPS);
    assertEquals(2.0, AllianceFlipUtil.applyY(2.0), EPS);
    Pose2d pose = new Pose2d(3.0, 2.0, Rotation2d.kZero);
    assertEquals(pose, AllianceFlipUtil.apply(pose));
  }

  @Test
  void redAllianceFlipsAcrossField() {
    setAllianceStation(AllianceStationID.Red1);
    assertTrue(AllianceFlipUtil.shouldFlip());
    assertEquals(FIELD_LENGTH - 3.0, AllianceFlipUtil.applyX(3.0), EPS);
    assertEquals(FIELD_WIDTH - 2.0, AllianceFlipUtil.applyY(2.0), EPS);

    Pose2d flipped = AllianceFlipUtil.apply(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    assertEquals(FIELD_LENGTH - 3.0, flipped.getX(), EPS);
    assertEquals(FIELD_WIDTH - 2.0, flipped.getY(), EPS);
    // A zero heading flipped for the red alliance points across the field (|theta| = pi).
    assertEquals(Math.PI, Math.abs(flipped.getRotation().getRadians()), EPS);
  }

  @Test
  void unknownAllianceDoesNotFlip() {
    // HalTestBase clears the alliance to Unknown before each test.
    assertFalse(AllianceFlipUtil.shouldFlip());
    assertEquals(5.0, AllianceFlipUtil.applyX(5.0), EPS);
    assertEquals(5.0, AllianceFlipUtil.applyY(5.0), EPS);
  }
}
