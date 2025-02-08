/*
 * MechanicalRuntimeException.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.util.exceptions;

/**
 * A {@code MechanicalRuntimeException} extends {@code RuntimeException} and is thrown if the robot
 * physically enters an error state. This can be used for minor errors or catastrophic errors.
 *
 * <p>{@code MechanicalRuntimeException} and its subclasses are <em>unchecked exceptions</em>. For
 * our purposes, this means that if the exception is not meant to be fatal, it should be handled
 * within a try/catch block.
 */
public class MechanicalRuntimeException extends RuntimeException {
  private static final String DEFAULT_MSG =
      """
        An error state exists on the physical
        robot. Check code permits an error
        state and try again.
        """;

  /** Constructs a {@code MechanicalRuntimeException} with the default message. */
  public MechanicalRuntimeException() {
    super(DEFAULT_MSG);
  }

  /**
   * Constructs a {@code MechanicalRuntimeException} with a simple string message. No additional
   * formatting is allowed.
   *
   * @param errorMessage String holding the error message.
   */
  public MechanicalRuntimeException(String errorMessage) {
    super(errorMessage);
  }

  /**
   * Constructs a {@code MechanicalRuntimeException} with a formatted string message. Use standard
   * format string parameters to format an error message. A newline character is automatically
   * added.
   *
   * @param errorMessageFmtString Format string holding the error message.
   * @param robotStateFields Variadic list of arguments that hydrates the format string.
   */
  public MechanicalRuntimeException(String errorMessageFmtString, Object... robotStateFields) {
    super(String.format(errorMessageFmtString, robotStateFields) + "\n");
  }
}
