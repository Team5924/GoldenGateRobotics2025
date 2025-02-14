/*
 * CoralInAndOutIOKrakenFOC.java
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

package org.team5924.frc2025.subsystems.rollers.CoralInAndOut;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;
import org.team5924.frc2025.util.LaserCAN_Measurement;
import org.team5924.frc2025.util.exceptions.SensorRuntimeException;

public class CoralInAndOutIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements CoralInAndOutIO {
  private static final int inNOutId = Constants.CORAL_IN_AND_OUT_CAN_ID;
  // private static final int handoffId = Constants.CORAL_HANDOFF_CAN_ID;
  private static final String bus = Constants.CORAL_IN_AND_OUT_BUS;
  private static final int currentLimitAmps = Constants.CORAL_IN_AND_OUT_CURRENT_LIMIT;
  private static final boolean invert = Constants.CORAL_IN_AND_OUT_INVERT;
  private static final boolean brake = Constants.CORAL_IN_AND_OUT_BRAKE;
  private static final double reduction = Constants.CORAL_IN_AND_OUT_REDUCTION;

  private static final LaserCan intakeLC = new LaserCan(Constants.CORAL_INTAKE_LASER_CAN_ID);
  private static final LaserCan shooterLC = new LaserCan(Constants.CORAL_SHOOTER_LASER_CAN_ID);

  private static final Alert intakeLCDisconnectAlert =
      new Alert("Intake LaserCAN disconnected.", AlertType.kWarning);
  private static final Alert shooterLCDisconnectAlert =
      new Alert("Shooter LaserCAN disconnected.", AlertType.kWarning);

  private static final Alert intakeLCInvalidMeasure =
      new Alert("Intake LaserCAN grabbed invalid measurement. See logs.", AlertType.kWarning);
  private static final Alert shooterLCInvalidMeasure =
      new Alert("Shooter LaserCAN grabbed invalid measurement. See logs.", AlertType.kWarning);

  public CoralInAndOutIOKrakenFOC() {
    super(inNOutId, bus, currentLimitAmps, invert, brake, reduction);
  }

  public void updateInputs(CoralInAndOutIOInputs inputs) {
    try {
      inputs.intakeLCMeasurement = LaserCAN_Measurement.fromLaserCAN(intakeLC.getMeasurement());
      inputs.intakeLCConnected = true;
      intakeLCDisconnectAlert.set(false);
      intakeLCInvalidMeasure.set(false);

    } catch (SensorRuntimeException e) {
      switch (e.getErrorType()) {
        case DISCONNECTED -> {
          inputs.intakeLCConnected = false;
          intakeLCDisconnectAlert.set(true);
        }
        case INVALID_DATA -> intakeLCInvalidMeasure.set(true);
        default -> {
          if (Constants.ALLOW_ASSERTS) throw e;
          else System.err.println("FIX NOW: Unhandled SensorRuntimeException: " + e.getMessage());
        }
      }
    }

    try {
      inputs.shooterLCMeasurement = LaserCAN_Measurement.fromLaserCAN(shooterLC.getMeasurement());
      inputs.shooterLCConnected = true;
      shooterLCDisconnectAlert.set(false);
      shooterLCInvalidMeasure.set(false);
    } catch (SensorRuntimeException e) {
      switch (e.getErrorType()) {
        case DISCONNECTED -> {
          inputs.shooterLCConnected = false;
          shooterLCDisconnectAlert.set(true);
        }
        case INVALID_DATA -> shooterLCInvalidMeasure.set(true);
        default -> {
          if (Constants.ALLOW_ASSERTS) throw e;
          else System.err.println("FIX NOW: Unhandled SensorRuntimeException: " + e.getMessage());
        }
      }
    }

    super.updateInputs(inputs);
  }
}
