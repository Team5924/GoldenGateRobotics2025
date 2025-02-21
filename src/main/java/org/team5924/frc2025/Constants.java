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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static final boolean ALLOW_ASSERTS = true;

  /* ### Subsystems ### */
  /* # Rollers # */
  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;
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
  /* # Pivot # */
  public static final int ALGAE_PIVOT_TALON_ID = 55;
  public static final double MOTOR_TO_ALGAE_PIVOT_REDUCTION = 3;
  public static final boolean TUNING_MODE = true;
  public static final int ALGAE_PIVOT_CANCODER_ID = 45;

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final Translation2d FIELD_CENTER =
      new Translation2d(
          Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldLength()),
          Meters.of(APRIL_TAG_FIELD_LAYOUT.getFieldWidth()));

  // Distance in one dimension from center to scoring poses in meters. Smaller number are the closer
  // positions
  public static final double SCORING_X_1 = 3.5051746;
  public static final double SCORING_X_2 = 3.7526976;
  public static final double SCORING_X_3 = 4.0372792;
  public static final double SCORING_X_4 = 4.5322998;
  public static final double SCORING_X_5 = 4.8168814;
  public static final double SCORING_X_6 = 5.0644044;

  // Only 3 because also mirrored up and down
  public static final double SCORING_Y_1 = 0.1642872;
  public static final double SCORING_Y_2 = 0.5929884;
  public static final double SCORING_Y_3 = 0.757301;

  public static final Pose2d SCORE_A_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_6), Meters.of(SCORING_Y_1))),
      new Rotation2d(Degrees.of(0)));
  public static final Pose2d SCORE_B_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_6), Meters.of(-SCORING_Y_1))),
      new Rotation2d(Degrees.of(0)));
  public static final Pose2d SCORE_C_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_5), Meters.of(-SCORING_Y_2))),
      new Rotation2d(Degrees.of(60)));
  public static final Pose2d SCORE_D_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_4), Meters.of(-SCORING_Y_3))),
      new Rotation2d(Degrees.of(60)));
  public static final Pose2d SCORE_E_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_3), Meters.of(-SCORING_Y_3))),
      new Rotation2d(Degrees.of(120)));
  public static final Pose2d SCORE_F_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_2), Meters.of(-SCORING_Y_2))),
      new Rotation2d(Degrees.of(120)));
  public static final Pose2d SCORE_G_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_1), Meters.of(-SCORING_Y_1))),
      new Rotation2d(Degrees.of(180)));
  public static final Pose2d SCORE_H_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_1), Meters.of(SCORING_Y_1))),
      new Rotation2d(Degrees.of(180)));
  public static final Pose2d SCORE_I_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_2), Meters.of(SCORING_Y_2))),
      new Rotation2d(Degrees.of(-120)));
  public static final Pose2d SCORE_J_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_3), Meters.of(SCORING_Y_3))),
      new Rotation2d(Degrees.of(-120)));
  public static final Pose2d SCORE_K_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_4), Meters.of(SCORING_Y_3))),
      new Rotation2d(Degrees.of(-60)));
  public static final Pose2d SCORE_L_BLUE =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(-SCORING_X_5), Meters.of(SCORING_Y_2))),
      new Rotation2d(Degrees.of(-60)));

  public static final Pose2d SCORE_A_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_6), Meters.of(-SCORING_Y_1))),
      new Rotation2d(Degrees.of(180)));
  public static final Pose2d SCORE_B_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_6), Meters.of(SCORING_Y_1))),
      new Rotation2d(Degrees.of(180)));
  public static final Pose2d SCORE_C_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_5), Meters.of(SCORING_Y_2))),
      new Rotation2d(Degrees.of(-120)));
  public static final Pose2d SCORE_D_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_4), Meters.of(SCORING_Y_3))),
      new Rotation2d(Degrees.of(-120)));
  public static final Pose2d SCORE_E_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_3), Meters.of(SCORING_Y_3))),
      new Rotation2d(Degrees.of(-60)));
  public static final Pose2d SCORE_F_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_2), Meters.of(SCORING_Y_2))),
      new Rotation2d(Degrees.of(-60)));
  public static final Pose2d SCORE_G_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_1), Meters.of(SCORING_Y_1))),
      new Rotation2d(Degrees.of(0)));
  public static final Pose2d SCORE_H_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_1), Meters.of(-SCORING_Y_1))),
      new Rotation2d(Degrees.of(0)));
  public static final Pose2d SCORE_I_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_2), Meters.of(-SCORING_Y_2))),
      new Rotation2d(Degrees.of(60)));
  public static final Pose2d SCORE_J_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_3), Meters.of(-SCORING_Y_3))),
      new Rotation2d(Degrees.of(60)));
  public static final Pose2d SCORE_K_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_4), Meters.of(-SCORING_Y_3))),
      new Rotation2d(Degrees.of(120)));
  public static final Pose2d SCORE_L_RED =
    new Pose2d(
      FIELD_CENTER.plus(new Translation2d(Meters.of(SCORING_X_5), Meters.of(-SCORING_Y_2))),
      new Rotation2d(Degrees.of(120)));
}
