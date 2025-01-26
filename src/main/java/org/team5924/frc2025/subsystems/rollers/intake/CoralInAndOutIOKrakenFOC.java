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
 * If this file has been seperated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025.subsystems.rollers.intake;

import au.grapplerobotics.LaserCan;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.subsystems.rollers.GenericRollerSystemIOKrakenFOC;
import org.team5924.frc2025.util.LaserCAN_Measurement;
import org.team5924.frc2025.util.LoggedTunableNumber;

public class CoralInAndOutIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements CoralInAndOutIO {
  private static final int id = Constants.CORAL_IN_AND_OUT_CAN_ID;
  private static final String bus = Constants.CORAL_IN_AND_OUT_BUS;
  private static final int currentLimitAmps = Constants.CORAL_IN_AND_OUT_CURRENT_LIMIT;
  private static final boolean invert = Constants.CORAL_IN_AND_OUT_INVERT;
  private static final boolean brake = Constants.CORAL_IN_AND_OUT_BRAKE;
  private static final double reduction = Constants.CORAL_IN_AND_OUT_REDUCTION;

  private static final LaserCan intakeLC = new LaserCan(Constants.CORAL_INTAKE_LASER_CAN_ID);
  private static final LaserCan shooterLC = new LaserCan(Constants.CORAL_SHOOTER_LASER_CAN_ID);

  private static final LoggedTunableNumber intakeDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/IntakeLaserCAN/DetectThreshold", 20);

  private static final LoggedTunableNumber shooterDetectThreshold =
      new LoggedTunableNumber("CoralInAndOutKrakenFOC/ShooterLaserCAN/DetectThreshold", 20);

  public CoralInAndOutIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }

  public void updateInputs(CoralInAndOutIOInputs inputs) {
    inputs.intakeLCMeasurement = LaserCAN_Measurement.fromLaserCAN(intakeLC.getMeasurement());
    inputs.shooterLCMeasurement = LaserCAN_Measurement.fromLaserCAN(shooterLC.getMeasurement());
    super.updateInputs(inputs);
  }

  public boolean isCoralInIntake() {
    return intakeLC.getMeasurement().distance_mm < (int) Math.floor(intakeDetectThreshold.get());
  }

  public boolean isCoralInShooter() {
    return shooterLC.getMeasurement().distance_mm < (int) Math.floor(shooterDetectThreshold.get());
  }
}
