// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Transform3d;
import org.frc5010.common.drive.pose.PoseProvider.ProviderType;
import org.junit.jupiter.api.Test;

/** Tests for {@link YaLLCamera}. */
public class YaLLCameraTest {

  @Test
  public void testCreateYaLLCamera() {
    YaLLCamera camera = new YaLLCamera("limelight-test", 0, new Transform3d());
    assertNotNull(camera);
    assertEquals(ProviderType.FIELD_BASED, camera.getType());
  }
}
