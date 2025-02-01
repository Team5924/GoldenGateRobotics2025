/*
 * SensorRuntimeException.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been seperated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.util.exceptions;

import lombok.Getter;

/**
 * A {@code SensorRuntimeException} extends {@code RuntimeException} and is thrown if the robot
 * physically enters an error state. This can be used for minor errors or catastrophic errors.
 *
 * <p>{@code SensorRuntimeException} and its subclasses are <em>unchecked exceptions</em>. For our
 * purposes, this means that if the exception is not meant to be fatal, it should be handled within
 * a try/catch block.
 */
public class SensorRuntimeException extends RuntimeException {
  public static enum SensorErrorType {
    DISCONNECTED,
    INVALID_DATA,
    INVALID_CONFIG,
    UNKNOWN
  }

  @Getter private final SensorErrorType errorType;

  private static final String DEFAULT_MSG =
      """
        An error state exists with a sensor connected to the robot.
        Check sensor connection and/or configuration and try again.
        """;

  /** Constructs a {@code SensorRuntimeException} with the default message. */
  public SensorRuntimeException() {
    this(SensorErrorType.UNKNOWN, DEFAULT_MSG);
  }

  /**
   * Constructs a {@code SensorRuntimeException} with a simple string message. No additional
   * formatting is allowed.
   *
   * @param errorMessage String holding the error message.
   */
  public SensorRuntimeException(SensorErrorType errorType, String errorMessage) {
    super(errorMessage);
    this.errorType = errorType;
  }

  /**
   * Constructs a {@code SensorRuntimeException} with a formatted string message. Use standard
   * format string parameters to format an error message. A newline character is automatically
   * added.
   *
   * @param errorMessageFmtString Format string holding the error message.
   * @param robotStateFields Variadic list of arguments that hydrates the format string.
   */
  public SensorRuntimeException(
      SensorErrorType errorType, String errorMessageFmtString, Object... robotStateFields) {
    super(String.format(errorMessageFmtString, robotStateFields) + "\n");
    this.errorType = errorType;
  }
}
