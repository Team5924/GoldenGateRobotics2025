/*
 * CoralInAndOutIO.java
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

package org.team5924.frc2025.subsystems.rollers.intake;

import au.grapplerobotics.interfaces.LaserCanInterface;
import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIO;
import org.team5924.frc2025.util.LaserCAN_ROI;

public interface CoralInAndOutIO extends GenericRollerSystemIO {

  @AutoLog
  abstract class CoralInAndOutIOInputs extends GenericRollerSystemIOInputs {
    /* Intake LaserCAN */
    public boolean intakeLCConnected = true;
    public int intakeLCStatus = LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
    public int intakeLCDistance = 0; // Distance in millimeters
    public int intakeLCAmbientLight = 0; // Ambient light measurement
    public boolean intakeLCIsLong = false; // Measurement taken with "long" mode
    public int intakeLCBudget = 0; // Budget used for measurement in milliseconds
    public LaserCAN_ROI intakeLCROI =
        new LaserCAN_ROI(0, 0, 0, 0); // Region of interest for measurement

    // Shooter LaserCAN
    public boolean shooterLCConnected = true;
    // LaserCan.Measurement shooterLCDistance = null;
  }
}
