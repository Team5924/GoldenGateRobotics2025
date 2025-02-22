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
  }

  private void setLLSettings() {
    LimelightHelpers.setPipelineIndex(
        "front",
        RobotState.getInstance().getIsRedAlliance()
            ? Constants.LIMELIGHT_RED_ALLIANCE_PIPELINE
            : Constants.LIMELIGHT_BLUE_ALLIANCE_PIPELINE);

    LimelightHelpers.setPipelineIndex(
        "back",
        RobotState.getInstance().getIsRedAlliance()
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
    } else {
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
    }
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

      inputs.megatag2PoseEstimatesFront = MegatagPoseEstimate.fromLimelight(megatag2Front);
      inputs.frontFiducials = FiducialObservation.fromLimelight(megatag2Front.rawFiducials);

      for (FiducialObservation rawFiducial : inputs.frontFiducials) {
        double frontTagAmbiguity = rawFiducial.ambiguity;
        if (frontTagAmbiguity < lowestTagAmbiguityFront) {
          lowestTagAmbiguityFront = frontTagAmbiguity;
        }
      }

      inputs.lowestTagAmbiguityFront = lowestTagAmbiguityFront;
    }

    if (inputs.backLimelightSeesTarget) {
      LimelightHelpers.PoseEstimate megatag2Back =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

      inputs.megatag2PoseEstimatesBack = MegatagPoseEstimate.fromLimelight(megatag2Back);
      inputs.backFiducials = FiducialObservation.fromLimelight(megatag2Back.rawFiducials);

      for (FiducialObservation rawFiducial : inputs.backFiducials) {
        double backTagAmbiguity = rawFiducial.ambiguity;
        if (backTagAmbiguity < lowestTagAmbiguityBack) {
          lowestTagAmbiguityBack = backTagAmbiguity;
        }
      }

      inputs.lowestTagAmbiguityBack = lowestTagAmbiguityBack;
    }

    inputs.aprilTagPipelineLatencySeconds =
        LimelightHelpers.getLatency_Pipeline(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;
    inputs.aprilTagCaptureLatencySeconds =
        LimelightHelpers.getLatency_Capture(Constants.APRIL_TAG_LIMELIGHT_NAME_FRONT) / 1000;

    setLLSettings();
  }
}
