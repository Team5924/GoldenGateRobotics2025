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

  /* Algae Rollers*/
  public static final int ALGAE_TALON_ID = 70;
  public static final String ALGAE_BUS = "rio";
  public static final int ALGAE_CURRENT_LIMIT = 40; // Adjust value as needed
  public static final boolean ALGAE_INVERT = false; // Adjust value as needed
  public static final boolean ALGAE_BRAKE = true; // Adjust value as needed
  public static final double ALGAE_REDUCTION = 1.0; // Adjust value as needed

  /* # Vision # */
  public static String APRIL_TAG_LIMELIGHT_NAME_FRONT = "limelight-front";
  public static String APRIL_TAG_LIMELIGHT_NAME_BACK = "limelight-back";

  public static final double FRONT_UP_LIMELIGHT_OFF_FORWARD = Meters.convertFrom(9.158, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(10.200, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_UP = Meters.convertFrom(11.202, Inches);
  public static final double FRONT_UP_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double FRONT_UP_LIMELIGHT_OFF_PITCH = -5.0;
  public static final double FRONT_UP_LIMELIGHT_OFF_YAW = 0.0;

  public static final double BACK_LIMELIGHT_OFF_FORWARD = -1 * Meters.convertFrom(-8.781, Inches);
  public static final double BACK_LIMELIGHT_OFF_SIDE = -1 * Meters.convertFrom(11.25, Inches);
  public static final double BACK_LIMELIGHT_OFF_UP = Meters.convertFrom(25.955, Inches);
  public static final double BACK_LIMELIGHT_OFF_ROLL = 0.0;
  public static final double BACK_LIMELIGHT_OFF_PITCH = 45.0;
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
    public static final double fieldWidth =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth();
    public static final Translation2d blueCenter =
        new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(159.28));

    public static final Translation2d redCenter =
        new Translation2d(Units.inchesToMeters(513.88), Units.inchesToMeters(159.03));

    public static final List<List<Pose2d>> branchRight2d = new ArrayList<>();
    public static final List<List<Pose2d>> branchLeft2d = new ArrayList<>();

    // public static final List<List<Pose2d>> redBranchRight2d = new ArrayList<>();
    // public static final List<List<Pose2d>> redBranchLeft2d = new ArrayList<>();

    static {
      double halfIsoBaseOfBranchesAndCenter = 0.120; //  Leg 1 (meters)
      double distanceFromCenterToRoboCenterLineup = 2.05; // Leg 3 (meters)
      double distanceFromCenterToRoboCenterShoot = 1.22; // Leg 3 but different (meters)
      double offset = -.205;
      double offsetCorrection = 0.09; // Correction for the offset just in case!!

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Pose2d poseDirection = new Pose2d(blueCenter, Rotation2d.fromDegrees(180 - (60 * face)));
        Logger.recordOutput("FacePoses/" + face, poseDirection);

        double radiusLineupCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterLineup, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        double radiusShootCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        var blueLeftBranchShoot =
            new Pose2d(
                blueCenter.getX()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(
                                    halfIsoBaseOfBranchesAndCenter + offset + offsetCorrection, 2))
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter + offset + offsetCorrection)
                                        / distanceFromCenterToRoboCenterShoot))),
                blueCenter.getY()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(
                                    halfIsoBaseOfBranchesAndCenter + offset + offsetCorrection, 2))
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter + offset + offsetCorrection)
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var blueRightBranchShoot =
            new Pose2d(
                blueCenter.getX()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter - offset, 2))
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter - offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                blueCenter.getY()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter - offset, 2))
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter - offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var blueLeftBranchLineup =
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

        var blueRightBranchLineup =
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
        rightBranch.add(blueRightBranchLineup);
        rightBranch.add(blueRightBranchShoot);
        Logger.recordOutput("ShootPosesRight/" + face, blueRightBranchShoot);

        ArrayList<Pose2d> leftBranch = new ArrayList<>();
        leftBranch.add(blueLeftBranchLineup);
        leftBranch.add(blueLeftBranchShoot);
        Logger.recordOutput("ShootPosesLeft/" + face, blueLeftBranchShoot);

        branchRight2d.add(rightBranch);
        branchLeft2d.add(leftBranch);
      }

      for (int face = 0; face < 6; face++) {
        Pose2d poseDirection = new Pose2d(redCenter, Rotation2d.fromDegrees(180 - (60 * face)));
        Logger.recordOutput("FacePoses/" + face, poseDirection);

        double radiusLineupCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterLineup, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        double radiusShootCircle =
            Math.sqrt(
                Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                    + Math.pow(halfIsoBaseOfBranchesAndCenter, 2));

        var redLeftBranchShoot =
            new Pose2d(
                redCenter.getX()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter + offset, 2))
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter + offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                redCenter.getY()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter + offset, 2))
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter + offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var redRightBranchShoot =
            new Pose2d(
                redCenter.getX()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter - offset, 2))
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter - offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                redCenter.getY()
                    + (Math.sqrt(
                            Math.pow(distanceFromCenterToRoboCenterShoot, 2)
                                + Math.pow(halfIsoBaseOfBranchesAndCenter - offset, 2))
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    (halfIsoBaseOfBranchesAndCenter - offset)
                                        / distanceFromCenterToRoboCenterShoot))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var redLeftBranchLineup =
            new Pose2d(
                redCenter.getX()
                    + (radiusLineupCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                redCenter.getY()
                    + (radiusLineupCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                - Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        var redRightBranchLineup =
            new Pose2d(
                redCenter.getX()
                    + (radiusLineupCircle
                        * Math.cos(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                redCenter.getY()
                    + (radiusLineupCircle
                        * Math.sin(
                            poseDirection.getRotation().getRadians()
                                + Math.atan(
                                    halfIsoBaseOfBranchesAndCenter
                                        / distanceFromCenterToRoboCenterLineup))),
                Rotation2d.fromRadians(Math.PI / 3 * face).unaryMinus());

        ArrayList<Pose2d> rightBranch = new ArrayList<>();
        rightBranch.add(redRightBranchLineup);
        rightBranch.add(redRightBranchShoot);
        Logger.recordOutput("ShootPosesRight/" + face, redRightBranchShoot);

        ArrayList<Pose2d> leftBranch = new ArrayList<>();
        leftBranch.add(redLeftBranchLineup);
        leftBranch.add(redLeftBranchShoot);
        Logger.recordOutput("ShootPosesLeft/" + face, redLeftBranchShoot);

        branchRight2d.add(rightBranch);
        branchLeft2d.add(leftBranch);
      }
    }
  }
}
