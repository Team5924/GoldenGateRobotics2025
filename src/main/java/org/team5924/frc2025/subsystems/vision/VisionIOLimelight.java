/*
 * VisionIOLimelight.java
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

package org.team5924.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.LimelightHelpers;
import org.team5924.frc2025.util.MegatagPoseEstimate;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
  public VisionIOLimelight() {
    LimelightHelpers.SetIMUMode("limelight-front", 1);
    LimelightHelpers.SetIMUMode("limelight-back", 1);
    LimelightHelpers.SetIMUMode("limelight-front-down", 1);

    RobotState.getInstance().setLimelightImuMode(1);
  }

  private void setLLSettings() {
    LimelightHelpers.setPipelineIndex(
        "limelight-front",
        RobotState.getInstance().isRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setPipelineIndex(
        "limelight-back",
        RobotState.getInstance().isRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setPipelineIndex(
        "limelight-front-down",
        RobotState.getInstance().isRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-front",
        Constants.FRONT_LEFT_LIMELIGHT_OFF_FORWARD,
        Constants.FRONT_LEFT_LIMELIGHT_OFF_SIDE,
        Constants.FRONT_LEFT_LIMELIGHT_OFF_UP,
        Constants.FRONT_LEFT_LIMELIGHT_OFF_ROLL,
        Constants.FRONT_LEFT_LIMELIGHT_OFF_PITCH,
        Constants.FRONT_LEFT_LIMELIGHT_OFF_YAW);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-back",
        Constants.BACK_LIMELIGHT_OFF_FORWARD,
        Constants.BACK_LIMELIGHT_OFF_SIDE,
        Constants.BACK_LIMELIGHT_OFF_UP,
        Constants.BACK_LIMELIGHT_OFF_ROLL,
        Constants.BACK_LIMELIGHT_OFF_PITCH,
        Constants.BACK_LIMELIGHT_OFF_YAW);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-front-down",
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_FORWARD,
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_SIDE,
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_UP,
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_ROLL,
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_PITCH,
        Constants.FRONT_RIGHT_LIMELIGHT_OFF_YAW);

    if (!DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode("limelight-front", 2);
      LimelightHelpers.SetIMUMode("limelight-back", 2);
      LimelightHelpers.SetIMUMode("limelight-front-down", 2);

      RobotState.getInstance().setLimelightImuMode(2);
    }
    // } else {
    LimelightHelpers.SetRobotOrientation(
        "limelight-front",
        RobotState.getInstance().getYawPosition().getDegrees(),
        RobotState.getInstance().getYawVelocityRadPerSec(),
        0,
        0,
        0,
        0);

    LimelightHelpers.SetRobotOrientation(
        "limelight-back",
        RobotState.getInstance().getYawPosition().getDegrees(),
        RobotState.getInstance().getYawVelocityRadPerSec(),
        0,
        0,
        0,
        0);
    // }
    LimelightHelpers.SetRobotOrientation(
        "limelight-front-down",
        RobotState.getInstance().getYawPosition().getDegrees(),
        RobotState.getInstance().getYawVelocityRadPerSec(),
        0,
        0,
        0,
        0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double lowestTagAmbiguityFrontLeft = 1;
    double lowestTagAmbiguityBack = 1;
    double lowestTagAmbiguityFrontRight = 1;

    inputs.frontLeftLimelightSeesTarget = LimelightHelpers.getTV("limelight-front");
    inputs.backLimelightSeesTarget = LimelightHelpers.getTV("limelight-back");
    inputs.frontRightLimelightSeesTarget = LimelightHelpers.getTV("limelight-front-down");

    if (inputs.frontLeftLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2FrontLeft =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

      inputs.megatag2PoseEstimateFrontLeft =
          MegatagPoseEstimate.fromLimelight(megatag2FrontLeft, true);
      if (megatag2FrontLeft != null) {
        inputs.frontLeftFiducials =
            FiducialObservation.fromLimelight(megatag2FrontLeft.rawFiducials);
        inputs.megatag2PoseEstimateFrontLeftPose2d = megatag2FrontLeft.pose;
        inputs.megatag2PoseEstimateFrontLeftTagCount = megatag2FrontLeft.tagCount;
        inputs.megatag2PoseEstimateFrontLeftAvgTagArea = megatag2FrontLeft.avgTagArea;
      }

      double FrontLeftDistToCameraTotal = 0.0;
      for (FiducialObservation rawFiducial : inputs.frontLeftFiducials) {
        double FrontLeftTagAmbiguity = rawFiducial.ambiguity;
        if (FrontLeftTagAmbiguity < lowestTagAmbiguityFrontLeft) {
          lowestTagAmbiguityFrontLeft = FrontLeftTagAmbiguity;
        }

        FrontLeftDistToCameraTotal += rawFiducial.distance;
      }

      inputs.lowestTagAmbiguityFrontLeft = lowestTagAmbiguityFrontLeft;
      // inputs.frontAprilTagDistance = inputs.frontFiducials[0].distance;
    }

    if (inputs.backLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2Back =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

      inputs.megatag2PoseEstimateBack = MegatagPoseEstimate.fromLimelight(megatag2Back, false);
      if (megatag2Back != null) {
        inputs.backFiducials = FiducialObservation.fromLimelight(megatag2Back.rawFiducials);
        inputs.megatag2PoseEstimateBackPose2d = megatag2Back.pose;
        inputs.megatag2PoseEstimateBackTagCount = megatag2Back.tagCount;
        inputs.megatag2PoseEstimateBackAvgTagArea = megatag2Back.avgTagArea;
      }

      for (FiducialObservation rawFiducial : inputs.backFiducials) {
        double backTagAmbiguity = rawFiducial.ambiguity;
        if (backTagAmbiguity < lowestTagAmbiguityBack) {
          lowestTagAmbiguityBack = backTagAmbiguity;
        }
      }

      inputs.lowestTagAmbiguityBack = lowestTagAmbiguityBack;
    }

    if (inputs.frontRightLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2FrontRight =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front-right");

      inputs.megatag2PoseEstimateFrontRight =
          MegatagPoseEstimate.fromLimelight(megatag2FrontRight, true);
      if (megatag2FrontRight != null) {
        inputs.frontRightFiducials =
            FiducialObservation.fromLimelight(megatag2FrontRight.rawFiducials);
        inputs.megatag2PoseEstimateFrontRightPose2d = megatag2FrontRight.pose;
        inputs.megatag2PoseEstimateFrontRightTagCount = megatag2FrontRight.tagCount;
        inputs.megatag2PoseEstimateFrontRightAvgTagArea = megatag2FrontRight.avgTagArea;
      }

      double FrontRightDistToCameraTotal = 0.0;
      for (FiducialObservation rawFiducial : inputs.frontRightFiducials) {
        double FrontRightTagAmbiguity = rawFiducial.ambiguity;
        if (FrontRightTagAmbiguity < lowestTagAmbiguityFrontRight) {
          lowestTagAmbiguityFrontRight = FrontRightTagAmbiguity;
        }

        FrontRightDistToCameraTotal += rawFiducial.distance;
      }

      inputs.lowestTagAmbiguityFrontRight = lowestTagAmbiguityFrontRight;
      // inputs.frontAprilTagDistance = inputs.frontFiducials[0].distance;
    }

    inputs.frontLeftAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline("limelight-front") / 1000;
    inputs.frontLeftAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture("limelight-front") / 1000;

    inputs.backAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;
    inputs.backAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;

    inputs.frontRightAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;
    inputs.frontRightAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;

    setLLSettings();
  }
}
