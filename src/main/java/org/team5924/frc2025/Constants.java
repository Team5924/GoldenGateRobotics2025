/*
 * Constants.java
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

package org.team5924.frc2025;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean TUNING_MODE = false;
  public static final boolean ALLOW_ASSERTS = true;

  /* ### Subsystems ### */
  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;
  /* Climber */
  public static final int CLIMBER_CAN_ID = 0; // TODO: update ID
  public static final String CLIMBER_BUS = "rio";
  public static final int CLIMBER_CURRENT_LIMIT = 40;
  public static final boolean CLIMBER_INVERT = false;
  public static final boolean CLIMBER_BRAKE = true;
  public static final double CLIMBER_REDUCTION = 18.0 / 12.0;
  public static final double CLIMBER_SIM_MOI = 0.001;
  public static final double CLIMBER_MIN_RADS = -Math.PI / 2;
  public static final double CLIMBER_MAX_RADS = Math.PI / 2;
  public static final int CLIMBER_LASER_CAN_ID = 0; // TODO: update ID

  /* SHOOTER */
  public static final int SHOOTER_CAN_ID = 0;
  public static final String SHOOTER_BUS = "rio";
  public static final int SHOOTER_CURRENT_LIMIT = 40;
  public static final boolean SHOOTER_INVERT = false;
  public static final boolean SHOOTER_BRAKE = true;
  public static final double SHOOTER_REDUCTION = 18.0 / 12.0;
  public static final double SHOOTER_SIM_MOI = 0.001;

  /* # Rollers # */
  /* Coral In-And-Out */
  public static final int CORAL_IN_AND_OUT_CAN_ID = 25;
  public static final String CORAL_IN_AND_OUT_BUS = "rio";
  public static final int CORAL_IN_AND_OUT_CURRENT_LIMIT = 40;
  public static final boolean CORAL_IN_AND_OUT_INVERT = false;
  public static final boolean CORAL_IN_AND_OUT_BRAKE = true;
  public static final double CORAL_IN_AND_OUT_REDUCTION = 18.0 / 12.0;
  public static final double CORAL_IN_AND_OUT_SIM_MOI = 0.001;
  public static final int CORAL_INTAKE_LASER_CAN_ID = 10;
  public static final int CORAL_SHOOTER_LASER_CAN_ID = 11;
}
