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
    LimelightHelpers.SetIMUMode("limelight-front-up", 1);
    LimelightHelpers.SetIMUMode("limelight-back", 1);
    LimelightHelpers.SetIMUMode("limelight-front-down", 1);

    RobotState.getInstance().setLimelightImuMode(1);
  }

  private void setLLSettings() {
    LimelightHelpers.setPipelineIndex(
        "limelight-front-up",
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
        "limelight-front-up",
        Constants.FRONT_UP_LIMELIGHT_OFF_FORWARD,
        Constants.FRONT_UP_LIMELIGHT_OFF_SIDE,
        Constants.FRONT_UP_LIMELIGHT_OFF_UP,
        Constants.FRONT_UP_LIMELIGHT_OFF_ROLL,
        Constants.FRONT_UP_LIMELIGHT_OFF_PITCH,
        Constants.FRONT_UP_LIMELIGHT_OFF_YAW);

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
        Constants.FRONT_DOWN_LIMELIGHT_OFF_FORWARD,
        Constants.FRONT_DOWN_LIMELIGHT_OFF_SIDE,
        Constants.FRONT_DOWN_LIMELIGHT_OFF_UP,
        Constants.FRONT_DOWN_LIMELIGHT_OFF_ROLL,
        Constants.FRONT_DOWN_LIMELIGHT_OFF_PITCH,
        Constants.FRONT_DOWN_LIMELIGHT_OFF_YAW);

    if (!DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode("limelight-front-up", 2);
      LimelightHelpers.SetIMUMode("limelight-back", 2);
      LimelightHelpers.SetIMUMode("limelight-front-down", 2);

      RobotState.getInstance().setLimelightImuMode(2);
    }
    // } else {
    LimelightHelpers.SetRobotOrientation(
        "limelight-front-up",
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
    double lowestTagAmbiguityFrontUp = 1;
    double lowestTagAmbiguityBack = 1;
    double lowestTagAmbiguityFrontDown = 1;

    inputs.frontUpLimelightSeesTarget = LimelightHelpers.getTV("limelight-front-up");
    inputs.backLimelightSeesTarget = LimelightHelpers.getTV("limelight-back");
    inputs.backLimelightSeesTarget = LimelightHelpers.getTV("limelight-front-down");

    if (inputs.frontUpLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2FrontUp =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front-up");

      inputs.megatag2PoseEstimateFrontUp = MegatagPoseEstimate.fromLimelight(megatag2FrontUp, true);
      if (megatag2FrontUp != null) {
        inputs.frontUpFiducials = FiducialObservation.fromLimelight(megatag2FrontUp.rawFiducials);
        inputs.megatag2PoseEstimateFrontUpPose2d = megatag2FrontUp.pose;
        inputs.megatag2PoseEstimateFrontUpTagCount = megatag2FrontUp.tagCount;
        inputs.megatag2PoseEstimateFrontUpAvgTagArea = megatag2FrontUp.avgTagArea;
      }

      double frontUpDistToCameraTotal = 0.0;
      for (FiducialObservation rawFiducial : inputs.frontUpFiducials) {
        double frontUpTagAmbiguity = rawFiducial.ambiguity;
        if (frontUpTagAmbiguity < lowestTagAmbiguityFrontUp) {
          lowestTagAmbiguityFrontUp = frontUpTagAmbiguity;
        }

        frontUpDistToCameraTotal += rawFiducial.distance;
      }

      inputs.frontUpAprilTagDistance = LimelightHelpers.getBotPose3d_TargetSpace("limelight-front-up");

      inputs.lowestTagAmbiguityFrontUp = lowestTagAmbiguityFrontUp;
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
      inputs.backAprilTagDistance = LimelightHelpers.getBotPose3d_TargetSpace("limelight-back");
    }

    if (inputs.frontDownLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2FrontDown =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front-up");

      inputs.megatag2PoseEstimateFrontDown = MegatagPoseEstimate.fromLimelight(megatag2FrontDown, true);
      if (megatag2FrontDown != null) {
        inputs.frontDownFiducials = FiducialObservation.fromLimelight(megatag2FrontDown.rawFiducials);
        inputs.megatag2PoseEstimateFrontDownPose2d = megatag2FrontDown.pose;
        inputs.megatag2PoseEstimateFrontDownTagCount = megatag2FrontDown.tagCount;
        inputs.megatag2PoseEstimateFrontDownAvgTagArea = megatag2FrontDown.avgTagArea;
      }

      double frontDownDistToCameraTotal = 0.0;
      for (FiducialObservation rawFiducial : inputs.frontDownFiducials) {
        double frontDownTagAmbiguity = rawFiducial.ambiguity;
        if (frontDownTagAmbiguity < lowestTagAmbiguityFrontDown) {
          lowestTagAmbiguityFrontDown = frontDownTagAmbiguity;
        }

        frontDownDistToCameraTotal += rawFiducial.distance;
      }

      inputs.frontDownAprilTagDistance = LimelightHelpers.getBotPose3d_TargetSpace("limelight-front-up");

      inputs.lowestTagAmbiguityFrontDown = lowestTagAmbiguityFrontDown;
      // inputs.frontAprilTagDistance = inputs.frontFiducials[0].distance;
    }

    inputs.frontUpAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;
    inputs.frontUpAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;

    inputs.backAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;
    inputs.backAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;

    inputs.frontDownAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;
    inputs.frontDownAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;

    setLLSettings();
  }
}
