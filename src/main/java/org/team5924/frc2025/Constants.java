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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
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
  public static final boolean ALLOW_ASSERTS = false;

  /* Field */
  public static final double FIELD_BORDER_MARGIN = 0.5;
  public static final AprilTagFieldLayout field =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final double FIELD_WIDTH = field.getFieldWidth();
  public static final double FIELD_LENGTH = field.getFieldLength();
  public static final double CORAL_STATION_RADIANS_NORMAL = 0.959931;

  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;
  /* Climber */
  public static final int CLIMBER_CAN_ID = 40;
  public static final String CLIMBER_BUS = "Drive CANivore";
  public static final int CLIMBER_CURRENT_LIMIT = 40;
  public static final InvertedValue CLIMBER_INVERT = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue CLIMBER_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final double CLIMBER_REDUCTION = 58.3; // TODO: correct for now, check again later
  public static final double CLIMBER_SIM_MOI = 0.001;
  public static final double CLIMBER_MIN_RADS = -Math.PI / 2; // TODO: get real min
  public static final double CLIMBER_MAX_RADS = Math.PI / 2; // TODO: get real max
  public static final int CLIMBER_LASER_CAN_ID = 42;

  /* # Rollers # */
  /* Coral In-And-Out */
  public static final int CORAL_IN_AND_OUT_CAN_ID = 33;
  public static final String CORAL_IN_AND_OUT_BUS = "rio";
  public static final int CORAL_IN_AND_OUT_CURRENT_LIMIT = 40;
  public static final boolean CORAL_IN_AND_OUT_INVERT = false;
  public static final boolean CORAL_IN_AND_OUT_BRAKE = true;
  public static final double CORAL_IN_AND_OUT_REDUCTION = 24.0 / 12.0;
  public static final double CORAL_IN_AND_OUT_SIM_MOI = 0.001;
  public static final int CORAL_INTAKE_LASER_CAN_ID = 10;
  public static final int CORAL_SHOOTER_LASER_CAN_ID = 11;

  /* # Pivot # */
  public static final int ALGAE_PIVOT_TALON_ID = 34;
  public static final double MOTOR_TO_ALGAE_PIVOT_REDUCTION = 62.5; // TODO: most likely change
  public static final int ALGAE_PIVOT_CANCODER_ID = 45;

  /* Coral Handoff */
  public static final int CORAL_HANDOFF_CAN_ID = 32;
  public static final String CORAL_HANDOFF_BUS = "rio";
  public static final int CORAL_HANDOFF_CURRENT_LIMIT = 40;
  public static final boolean CORAL_HANDOFF_INVERT = false;
  public static final boolean CORAL_HANDOFF_BRAKE = false;
  public static final double CORAL_HANDOFF_REDUCTION = 24.0 / 12.0;
  public static final double CORAL_HANDOFF_SIM_MOI = 0.001;
  // TODO: Fill out Coral Handoff Constants with real values - all need to be fixed

  /* # Elevator # */
  public static final int ELEVATOR_LEFT_TALON_ID = 30;
  public static final int ELEVATOR_RIGHT_TALON_ID = 31;
  public static final int ELEVATOR_CANCODER_ID = 40; // TODO: Check and change if needed
  public static final int ELEVATOR_CANDI_ID = 39;
  public static final String ELEVATOR_CANDI_BUS = "rio";
  public static final double MOTOR_TO_ELEVATOR_REDUCTION = 4.00;
  public static final double CANCODER_TO_ELEVATOR_REDUCTION = 1.0;
  public static final InvertedValue ELEVATOR_LEFT_INVERSION = InvertedValue.Clockwise_Positive;
  public static final Distance SPROCKET_RADIUS = Inches.of(.6405);
  public static final double ELEVATOR_CANCODER_OFFSET = 0.00; // TODO: Check and change if needed

  /* Algae Rollers*/
  public static final int ALGAE_TALON_ID = 35;
  public static final String ALGAE_BUS = "rio";
  public static final int ALGAE_CURRENT_LIMIT = 40; // Adjust value as needed
  public static final boolean ALGAE_INVERT = false; // Adjust value as needed
  public static final boolean ALGAE_BRAKE = true; // Adjust value as needed
  public static final double ALGAE_REDUCTION = 1.0; // Adjust value as needed

  /* # Vision # */
  public static String APRIL_TAG_LIMELIGHT_NAME_FRONT = "limelight-front";
  public static String APRIL_TAG_LIMELIGHT_NAME_BACK = "limelight-back";

  public static final double FRONT_UP_LIMELIGHT_OFF_FORWARD = Meters.convertFrom(8.885, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(9.755, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_UP = Meters.convertFrom(16.17, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double FRONT_UP_LIMELIGHT_OFF_PITCH = 5.0;
  public static final double FRONT_UP_LIMELIGHT_OFF_YAW = 0.0;

  public static final double BACK_LIMELIGHT_OFF_FORWARD = -1 * Meters.convertFrom(8.971, Inches);
  public static final double BACK_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(9.755, Inches);
  public static final double BACK_LIMELIGHT_OFF_UP = Meters.convertFrom(16.145, Inches);
  public static final double BACK_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double BACK_LIMELIGHT_OFF_PITCH = 15.0;
  public static final double BACK_LIMELIGHT_OFF_YAW = 180.0;

  public static final double FRONT_DOWN_LIMELIGHT_OFF_FORWARD =
      -1 * Meters.convertFrom(8.971, Inches);
  public static final double FRONT_DOWN_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(9.755, Inches);
  public static final double FRONT_DOWN_LIMELIGHT_OFF_UP = Meters.convertFrom(16.145, Inches);
  public static final double FRONT_DOWN_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double FRONT_DOWN_LIMELIGHT_OFF_PITCH = 15.0;
  public static final double FRONT_DOWN_LIMELIGHT_OFF_YAW = 180.0;

  public static final int LIMELIGHT_RED_ALLIANCE_PIPELINE = 0;
  public static final int LIMELIGHT_BLUE_ALLIANCE_PIPELINE = 0;

  public static final Distance ROBOT_LENGTH_WITH_BUMPERS_FRONT_TO_BACK = Inches.of(35.75);
  public static final Distance ROBOT_LENGTH_WITH_BUMPERS_LEFT_TO_RIGHT = Inches.of(36.5);
  public static final Distance DELTA_X_CENTER_OF_CORAL_OUT_FROM_CENTER = Inches.of(-5.5);

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final double fieldWidth = field.getFieldWidth();
    public static final Translation2d blueCenter =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);

    public static final Translation2d redCenter =
        new Translation2d(Units.inchesToMeters(513.88), fieldWidth / 2.0);

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Pose2d> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    public static final List<List<Pose2d>> branchRight2d = new ArrayList<>();
    public static final List<List<Pose2d>> branchLeft2d = new ArrayList<>();

    static {
      // Initialize faces
      centerFaces[0] = field.getTagPose(18).get().toPose2d();
      centerFaces[1] = field.getTagPose(19).get().toPose2d();
      centerFaces[2] = field.getTagPose(20).get().toPose2d();
      centerFaces[3] = field.getTagPose(21).get().toPose2d();
      centerFaces[4] = field.getTagPose(22).get().toPose2d();
      centerFaces[5] = field.getTagPose(17).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Pose2d poseDirectionBlue =
            new Pose2d(blueCenter, Rotation2d.fromDegrees(180 - (60 * face)));
        Pose2d poseDirectionRed = new Pose2d(redCenter, Rotation2d.fromDegrees(180 - (60 * face)));
        double adjustX = Units.inchesToMeters(30.738); // robot x
        double adjustYLeft = Units.inchesToMeters(6.469); // robot y left
        double adjustYRight = Units.inchesToMeters(6.469); // robot y right

        var rightBranchPoseShootBlue =
            new Pose2d(
                poseDirectionBlue
                    .transformBy(new Transform2d(adjustX, adjustYRight, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionBlue.getRotation().getRadians()));

        var leftBranchPoseShootBlue =
            new Pose2d(
                poseDirectionBlue
                    .transformBy(new Transform2d(adjustX, -adjustYLeft, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionBlue.getRotation().getRadians()));

        var rightBranchLineupPoseBlue =
            new Pose2d(
                poseDirectionBlue
                    .transformBy(new Transform2d(adjustX, adjustYRight, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionBlue.getRotation().getRadians()));

        var leftBranchLineupPoseBlue =
            new Pose2d(
                poseDirectionBlue
                    .transformBy(new Transform2d(adjustX, -adjustYLeft, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionBlue.getRotation().getRadians()));

        var rightBranchPoseShootRed =
            new Pose2d(
                poseDirectionRed
                    .transformBy(new Transform2d(adjustX, adjustYRight, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionRed.getRotation().getRadians()));

        var leftBranchPoseShootRed =
            new Pose2d(
                poseDirectionRed
                    .transformBy(new Transform2d(adjustX, -adjustYLeft, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionRed.getRotation().getRadians()));

        var rightBranchLineupPoseRed =
            new Pose2d(
                poseDirectionRed
                    .transformBy(new Transform2d(adjustX, adjustYRight, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionRed.getRotation().getRadians()));

        var leftBranchLineupPoseRed =
            new Pose2d(
                poseDirectionRed
                    .transformBy(new Transform2d(adjustX, -adjustYLeft, Rotation2d.kZero))
                    .getTranslation(),
                new Rotation2d(poseDirectionRed.getRotation().getRadians()));

        ArrayList<Pose2d> rightBranchBlue = new ArrayList<>();
        rightBranchBlue.add(rightBranchLineupPoseBlue);
        rightBranchBlue.add(rightBranchPoseShootBlue);
        Logger.recordOutput("ShootPosesRight/" + face, rightBranchPoseShootBlue);

        ArrayList<Pose2d> rightBranchRed = new ArrayList<>();
        rightBranchRed.add(rightBranchLineupPoseRed);
        rightBranchRed.add(rightBranchPoseShootRed);
        Logger.recordOutput("ShootPosesRight/" + face, rightBranchPoseShootRed);

        ArrayList<Pose2d> leftBranchBlue = new ArrayList<>();
        leftBranchBlue.add(leftBranchLineupPoseBlue);
        leftBranchBlue.add(leftBranchPoseShootBlue);
        Logger.recordOutput("ShootPosesLeft/" + face, leftBranchPoseShootBlue);

        ArrayList<Pose2d> leftBranchRed = new ArrayList<>();
        leftBranchRed.add(leftBranchLineupPoseRed);
        leftBranchRed.add(leftBranchPoseShootRed);
        Logger.recordOutput("ShootPosesLeft/" + face, leftBranchPoseShootRed);

        branchRight2d.add(rightBranchBlue);
        branchLeft2d.add(leftBranchBlue);

        branchRight2d.add(rightBranchRed);
        branchLeft2d.add(leftBranchRed);
      }
    }
  }
}
