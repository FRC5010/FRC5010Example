// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.units;

import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.lang.reflect.Method;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Pure-logic tests for the unit enums in {@code org.frc5010.common.config.units}.
 *
 * <p>Every enum follows the same contract: a {@code static fromString(String)} that maps any of the
 * constant's aliases (case-insensitively) to that constant, and throws {@link
 * IllegalArgumentException} for unknown or null input. This test verifies that contract
 * reflectively across all enums and all of their declared aliases, so adding a constant or alias is
 * automatically covered.
 */
class UnitEnumTest {

  /** All unit enums that expose {@code fromString(String)} and {@code getAliases()}. */
  static Stream<Class<? extends Enum<?>>> unitEnums() {
    return Stream.of(
        AngleUnit.class,
        AngularAccelerationUnit.class,
        AngularVelocityUnit.class,
        CurrentUnit.class,
        DistanceUnit.class,
        LinearAccelerationUnit.class,
        LinearVelocityUnit.class,
        MassUnit.class,
        MomentOfInertiaUnit.class,
        TimeUnit.class,
        VoltageUnit.class);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("unitEnums")
  void everyAliasResolvesToItsConstantCaseInsensitively(Class<? extends Enum<?>> enumClass)
      throws Exception {
    Method fromString = enumClass.getMethod("fromString", String.class);
    Method getAliases = enumClass.getMethod("getAliases");

    for (Enum<?> constant : enumClass.getEnumConstants()) {
      String[] aliases = (String[]) getAliases.invoke(constant);
      for (String alias : aliases) {
        assertSame(constant, fromString.invoke(null, alias), enumClass.getSimpleName() + ": " + alias);
        assertSame(
            constant,
            fromString.invoke(null, alias.toUpperCase()),
            enumClass.getSimpleName() + ": " + alias + " (upper)");
        assertSame(
            constant,
            fromString.invoke(null, "  " + alias + "  "),
            enumClass.getSimpleName() + ": " + alias + " (padded)");
      }
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("unitEnums")
  void unknownAndNullThrow(Class<? extends Enum<?>> enumClass) throws Exception {
    Method fromString = enumClass.getMethod("fromString", String.class);
    // Reflective invocation wraps thrown exceptions; unwrap and assert the cause type.
    assertThrows(
        IllegalArgumentException.class,
        () -> invokeUnwrapped(fromString, "definitely-not-a-real-unit"));
    assertThrows(IllegalArgumentException.class, () -> invokeUnwrapped(fromString, (Object) null));
  }

  private static void invokeUnwrapped(Method method, Object arg) throws Throwable {
    try {
      method.invoke(null, arg);
    } catch (java.lang.reflect.InvocationTargetException e) {
      throw e.getCause();
    }
  }
}
