/*
 * ElevatorIO.java
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

package org.team5924.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double rads) {}
}
