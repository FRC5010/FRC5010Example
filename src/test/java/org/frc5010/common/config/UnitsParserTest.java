// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.frc5010.common.config.json.UnitValueJson;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * Pure-logic tests for {@link UnitsParser}. These exercise unit-string conversion to WPILib measure
 * types with no hardware/HAL dependency. They pin the documented physical conversions and the
 * fail-on-unknown-unit behavior so future edits cannot silently change them.
 */
class UnitsParserTest {
  private static final double EPS = 1e-9;

  @ParameterizedTest(name = "{0} {1} -> {2} m")
  @CsvSource({
    "1.0, meters, 1.0",
    "1.0, m, 1.0",
    "1.0, ft, 0.3048",
    "1.0, feet, 0.3048",
    "1.0, in, 0.0254",
    "1.0, yd, 0.9144", // 1 yard = 3 feet = 0.9144 m
    "1000.0, mm, 1.0",
    "100.0, cm, 1.0",
  })
  void parseDistanceConvertsToMeters(double magnitude, String unit, double expectedMeters) {
    assertEquals(expectedMeters, UnitsParser.parseDistance(magnitude, unit).in(Meters), EPS);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} m/s")
  @CsvSource({
    "1.0, m/s, 1.0",
    "1000.0, mm/s, 1.0",
    "100.0, cm/s, 1.0",
    "1.0, ft/s, 0.3048",
  })
  void parseVelocityConvertsToMetersPerSecond(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseVelocity(magnitude, unit).in(MetersPerSecond), EPS);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} m/s^2")
  @CsvSource({
    "1.0, m/s^2, 1.0",
    "1.0, ft/s^2, 0.3048",
  })
  void parseAccelerationConvertsToMetersPerSecondSquared(
      double magnitude, String unit, double expected) {
    assertEquals(
        expected, UnitsParser.parseAccelleration(magnitude, unit).in(MetersPerSecondPerSecond), EPS);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} kg")
  @CsvSource({
    "1.0, kg, 1.0",
    "1000.0, g, 1.0",
    "1.0, stone, 6.35029",
    "1.0, tons, 1000.0",
  })
  void parseMassConvertsToKilograms(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseMass(magnitude, unit).in(Kilograms), 1e-6);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} rad")
  @CsvSource({
    "180.0, deg, 3.141592653589793",
    "1.0, rad, 1.0",
    "1.0, rotations, 6.283185307179586", // 2*pi
  })
  void parseAngleConvertsToRadians(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseAngle(magnitude, unit).in(Radians), 1e-9);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} rad/s")
  @CsvSource({
    "1.0, rad/s, 1.0",
    "1.0, rps, 6.283185307179586", // 1 rotation/s = 2*pi rad/s
    "60.0, rpm, 6.283185307179586", // 60 rpm = 1 rotation/s = 2*pi rad/s
  })
  void parseAngularVelocityConvertsToRadiansPerSecond(
      double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseAngularVelocity(magnitude, unit).in(RadiansPerSecond), 1e-9);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} rad/s^2")
  @CsvSource({
    "1.0, rad/s^2, 1.0",
    "1.0, rps^2, 6.283185307179586", // 1 rotation/s^2 = 2*pi rad/s^2
  })
  void parseAngularAccelerationConvertsToRadiansPerSecondSquared(
      double magnitude, String unit, double expected) {
    assertEquals(
        expected,
        UnitsParser.parseAngularAcceleration(magnitude, unit).in(RadiansPerSecondPerSecond),
        1e-9);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} s")
  @CsvSource({
    "1.0, s, 1.0",
    "1000.0, ms, 1.0",
    "1.0, min, 60.0",
  })
  void parseTimeConvertsToSeconds(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseTime(magnitude, unit).in(Seconds), EPS);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} V")
  @CsvSource({
    "1.0, volts, 1.0",
    "1000.0, mv, 1.0",
    "1.0, kv, 1000.0",
  })
  void parseVoltsConvertsToVolts(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseVolts(magnitude, unit).in(Volts), EPS);
  }

  @ParameterizedTest(name = "{0} {1} -> {2} A")
  @CsvSource({
    "1.0, amps, 1.0",
    "1000.0, ma, 1.0",
  })
  void parseAmpsConvertsToAmps(double magnitude, String unit, double expected) {
    assertEquals(expected, UnitsParser.parseAmps(magnitude, unit).in(Amps), EPS);
  }

  @Test
  void unitValueJsonOverloadMatchesPrimitiveOverload() {
    assertEquals(
        UnitsParser.parseDistance(2.0, "ft").in(Meters),
        UnitsParser.parseDistance(new UnitValueJson(2.0, "ft")).in(Meters),
        EPS);
  }

  @Test
  void unknownUnitThrows() {
    // fromString rejects unknown unit strings, so the parser surfaces an
    // IllegalArgumentException rather than silently returning a default.
    assertThrows(
        IllegalArgumentException.class, () -> UnitsParser.parseDistance(1.0, "furlong"));
    assertThrows(IllegalArgumentException.class, () -> UnitsParser.parseMass(1.0, "quintal"));
  }
}
