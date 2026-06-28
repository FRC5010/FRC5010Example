// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base class for tests that exercise code requiring the WPILib HAL (NetworkTables, DriverStation,
 * sim devices). It initializes the HAL once per test class and, before every test, resets the
 * simulated alliance to {@code Unknown} so DriverStation-dependent code is deterministic regardless
 * of test ordering (tests share a single JVM).
 */
public abstract class HalTestBase {

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "WPILib HAL failed to initialize");
  }

  @BeforeEach
  void clearAlliance() {
    setAllianceStation(AllianceStationID.Unknown);
  }

  @AfterAll
  static void resetAllianceAfterClass() {
    setAllianceStation(AllianceStationID.Unknown);
  }

  /**
   * Sets the simulated alliance station and lets {@link DriverStation} observe the change so that
   * {@code DriverStation.getAlliance()} reflects it immediately.
   *
   * @param id the alliance station to simulate
   */
  protected static void setAllianceStation(AllianceStationID id) {
    DriverStationSim.setAllianceStationId(id);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }
}
