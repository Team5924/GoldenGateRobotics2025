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

    RobotState.getInstance().setLimelightImuMode(1);
  }

  private void setLLSettings() {
    LimelightHelpers.setPipelineIndex(
        "front",
        RobotState.getInstance().isRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setPipelineIndex(
        "back",
        RobotState.getInstance().isRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-front",
        Constants.FRONT_LIMELIGHT_OFF_FORWARD,
        Constants.FRONT_LIMELIGHT_OFF_SIDE,
        Constants.FRONT_LIMELIGHT_OFF_UP,
        Constants.FRONT_LIMELIGHT_OFF_ROLL,
        Constants.FRONT_LIMELIGHT_OFF_PITCH,
        Constants.FRONT_LIMELIGHT_OFF_YAW);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-back",
        Constants.BACK_LIMELIGHT_OFF_FORWARD,
        Constants.BACK_LIMELIGHT_OFF_SIDE,
        Constants.BACK_LIMELIGHT_OFF_UP,
        Constants.BACK_LIMELIGHT_OFF_ROLL,
        Constants.BACK_LIMELIGHT_OFF_PITCH,
        Constants.BACK_LIMELIGHT_OFF_YAW);

    if (!DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode("limelight-front", 2);
      LimelightHelpers.SetIMUMode("limelight-back", 2);

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
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double lowestTagAmbiguityFront = 1;
    double lowestTagAmbiguityBack = 1;

    inputs.frontLimelightSeesTarget = LimelightHelpers.getTV("limelight-front");
    inputs.backLimelightSeesTarget = LimelightHelpers.getTV("limelight-back");

    if (inputs.frontLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2Front =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

      inputs.megatag2PoseEstimateFront = MegatagPoseEstimate.fromLimelight(megatag2Front, true);
      if (megatag2Front != null) {
        inputs.frontFiducials = FiducialObservation.fromLimelight(megatag2Front.rawFiducials);
        inputs.megatag2PoseEstimateFrontPose2d = megatag2Front.pose;
        inputs.megatag2PoseEstimateFrontTagCount = megatag2Front.tagCount;
        inputs.megatag2PoseEstimateFrontAvgTagArea = megatag2Front.avgTagArea;
      }

      double frontDistToCameraTotal = 0.0;
      for (FiducialObservation rawFiducial : inputs.frontFiducials) {
        double frontTagAmbiguity = rawFiducial.ambiguity;
        if (frontTagAmbiguity < lowestTagAmbiguityFront) {
          lowestTagAmbiguityFront = frontTagAmbiguity;
        }

        frontDistToCameraTotal += rawFiducial.distance;
      }

      inputs.frontAprilTagDistance = LimelightHelpers.getBotPose3d_TargetSpace("limelight-front");

      inputs.lowestTagAmbiguityFront = lowestTagAmbiguityFront;
      // inputs.frontAprilTagDistance = inputs.frontFiducials[0].distance;
    }

    if (inputs.backLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2Back =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

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
      inputs.backAprilTagDistance = megatag2Back.avgTagDist;
    }

    inputs.frontAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;
    inputs.frontAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;

    inputs.backAprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;
    inputs.backAprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_BACK) / 1000;

    setLLSettings();
  }
}
