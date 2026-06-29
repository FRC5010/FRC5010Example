// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

/**
 * Thrown when a robot configuration file is missing or cannot be parsed.
 *
 * <p>Unlike a bare {@code assert} (which is disabled at runtime on the robot) this surfaces a clear,
 * actionable message — typically including the offending file path — at the point the problem is
 * detected. Construction boundaries are expected to catch it and report it via {@link
 * edu.wpi.first.wpilibj.DriverStation#reportError} rather than let it crash and restart the robot
 * program.
 */
public class ConfigException extends RuntimeException {
  /**
   * Creates a new ConfigException.
   *
   * @param message a description of the configuration problem (include the file path)
   */
  public ConfigException(String message) {
    super(message);
  }

  /**
   * Creates a new ConfigException with an underlying cause.
   *
   * @param message a description of the configuration problem (include the file path)
   * @param cause the underlying exception
   */
  public ConfigException(String message, Throwable cause) {
    super(message, cause);
  }
}
