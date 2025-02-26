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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

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

  public static final boolean TUNING_MODE = true;
  public static final boolean ALLOW_ASSERTS = true;

  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;

  /* ### Subsystems ### */
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
  public static final int ALGAE_PIVOT_TALON_ID = 55;
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

  /* # Vision # */
  public static String APRIL_TAG_LIMELIGHT_NAME_FRONT = "limelight-front";
  public static String APRIL_TAG_LIMELIGHT_NAME_BACK = "limelight-back";

  public static final double FRONT_LIMELIGHT_OFF_FORWARD = Meters.convertFrom(8.885, Inches);
  public static final double FRONT_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(9.755, Inches);
  public static final double FRONT_LIMELIGHT_OFF_UP = Meters.convertFrom(16.17, Inches);
  public static final double FRONT_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double FRONT_LIMELIGHT_OFF_PITCH = 5.0;
  public static final double FRONT_LIMELIGHT_OFF_YAW = 0.0;

  public static final double BACK_LIMELIGHT_OFF_FORWARD = -1 * Meters.convertFrom(8.971, Inches);
  public static final double BACK_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(9.755, Inches);
  public static final double BACK_LIMELIGHT_OFF_UP = Meters.convertFrom(16.145, Inches);
  public static final double BACK_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double BACK_LIMELIGHT_OFF_PITCH = 15.0;
  public static final double BACK_LIMELIGHT_OFF_YAW = 180.0;

  public static final int LIMELIGHT_RED_ALLIANCE_PIPELINE = 0;
  public static final int LIMELIGHT_BLUE_ALLIANCE_PIPELINE = 0;

  public static final Distance ROBOT_LENGTH_WITH_BUMPERS_FRONT_TO_BACK = Inches.of(35.75);
  public static final Distance ROBOT_LENGTH_WITH_BUMPERS_LEFT_TO_RIGHT = Inches.of(36.5);
  public static final Distance DELTA_X_CENTER_OF_CORAL_OUT_FROM_CENTER = Inches.of(-5.5);

  //   public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
  //       AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final double height;
    public final double pitch;
  }

  public static class Reef {
    public static final double fieldLength =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldLength();
    public static final double fieldWidth =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth();
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d blueCenter =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<List<Pose2d>> branchRight2d = new ArrayList<>();
    public static final List<List<Pose2d>> branchLeft2d = new ArrayList<>();

    static {
      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Pose2d poseDirection = new Pose2d(blueCenter, Rotation2d.fromDegrees(180 - (60 * face)));
        Logger.recordOutput("FacePoses/" + face, poseDirection);

        double halfIsoBaseOfBranchesAndCenter = 0.120; //  Leg 1 (meters)
        double distanceFromCenterToRoboCenterLineup = 1.75; // Leg 3 (meters)
        double distanceFromCenterToRoboCenterShoot = 1.276; // Leg 3 but different (meters)

        double radiusLineupCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterLineup, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        double radiusShootCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        var leftBranchPost =
            new Pose2d(
                blueCenter.getX()
                    + (radiusShootCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterShoot))),
                blueCenter.getY()
                    + (radiusShootCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var rightBranchPost =
            new Pose2d(
                blueCenter.getX()
                    + (radiusShootCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterShoot))),
                blueCenter.getY()
                    + (radiusShootCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var leftBranchLineup =
            new Pose2d(
                blueCenter.getX()
                    + (radiusLineupCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                blueCenter.getY()
                    + (radiusLineupCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var rightBranchLineup =
            new Pose2d(
                blueCenter.getX()
                    + (radiusLineupCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                blueCenter.getY()
                    + (radiusLineupCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        ArrayList<Pose2d> rightBranch = new ArrayList<>();
        rightBranch.add(rightBranchLineup);
        rightBranch.add(rightBranchPost);

        ArrayList<Pose2d> leftBranch = new ArrayList<>();
        leftBranch.add(leftBranchLineup);
        leftBranch.add(leftBranchPost);

        branchRight2d.add(rightBranch);
        branchLeft2d.add(leftBranch);
      }
    }
  }
}
