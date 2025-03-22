/*
 * AlgaeRollerIOKrakenFOC.java
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

package org.team5924.frc2025.subsystems.rollers.algae;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;
import org.team5924.frc2025.util.LaserCAN_Measurement;
import org.team5924.frc2025.util.exceptions.SensorRuntimeException;

public class AlgaeRollerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements AlgaeRollerIO {

  private static final int AlgaeRollerId = Constants.ALGAE_TALON_ID;
  private static final String bus = Constants.ALGAE_BUS;
  private static final int currentLimitAmps = Constants.ALGAE_CURRENT_LIMIT;
  private static final boolean invert = Constants.ALGAE_INVERT;
  private static final boolean loadShootBrake = Constants.ALGAE_BRAKE;
  private static final double reduction = Constants.ALGAE_REDUCTION;

  private static final LaserCan AlgaeRollerLC = new LaserCan(Constants.CORAL_INTAKE_LASER_CAN_ID);

  private static final Alert AlgaeRollerLCDisconnectAlert =
      new Alert("Algae LaserCAN disconnected.", AlertType.kWarning);

  private static final Alert AlgaeRollerLCInvalidMeasure =
      new Alert("Algae LaserCAN grabbed invalid measurement. See logs.", AlertType.kWarning);

  public AlgaeRollerIOKrakenFOC() {
    super(AlgaeRollerId, bus, currentLimitAmps, invert, loadShootBrake, reduction);
  }

  public void updateInputs(AlgaeRollerIOInputs inputs) {
    try {
      inputs.algaeRollerLCMeasurement =
          LaserCAN_Measurement.fromLaserCAN(AlgaeRollerLC.getMeasurement());
      inputs.algaeRollerLCConnected = true;
      AlgaeRollerLCDisconnectAlert.set(false);
      AlgaeRollerLCInvalidMeasure.set(false);
    } catch (SensorRuntimeException e) {
      switch (e.getErrorType()) {
        case DISCONNECTED -> {
          inputs.algaeRollerLCConnected = false;
          AlgaeRollerLCDisconnectAlert.set(true);
        }
        case INVALID_DATA -> AlgaeRollerLCInvalidMeasure.set(true);
        default -> {
          if (Constants.ALLOW_ASSERTS) throw e;
          else System.err.println("FIX NOW: Unhandled SensorRuntimeException: " + e.getMessage());
        }
      }
    }

    super.updateInputs(inputs);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
