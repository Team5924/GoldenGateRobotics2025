/*
 * Vision.java
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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.MegatagPoseEstimate;
import org.team5924.frc2025.util.VisionFieldPoseEstimate;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;

  private double lastVisionTimestamp = 0;

  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  // private final BooleanSubscriber allianceSubscriber =
  //     NetworkTableInstance.getDefault()
  //         .getTable("FMSInfo")
  //         .getBooleanTopic("IsRedAlliance")
  //         .subscribe(true);
  // private boolean previousAllianceSubscriberValue = true;

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    updateVision(
        inputs.frontLeftLimelightSeesTarget,
        inputs.frontLeftFiducials,
        inputs.megatag2PoseEstimateFrontLeft,
        false);

    updateVision(
        inputs.frontRightLimelightSeesTarget,
        inputs.frontRightFiducials,
        inputs.megatag2PoseEstimateFrontRight,
        true);

    updateVision(
        inputs.backLimelightSeesTarget,
        inputs.backFiducials,
        inputs.megatag2PoseEstimateBack,
        false);

    // boolean isRedAlliance = allianceSubscriber.get();
    // if (isRedAlliance != previousAllianceSubscriberValue) {
    //   previousAllianceSubscriberValue = isRedAlliance;
    //   RobotState.getInstance().setRedAlliance(isRedAlliance);
    // }
  }

  private void updateVision(
      boolean cameraSeesTarget,
      FiducialObservation[] cameraFiducialObservations,
      MegatagPoseEstimate megatag2PoseEstimate,
      Boolean isFrontRightLL) {
    if (megatag2PoseEstimate != null) {
      // System.out.println("Megatag2PoseEstimate is not null");
      boolean filterOut =
          megatag2PoseEstimate.pose.getX() < -Constants.FIELD_BORDER_MARGIN
              || megatag2PoseEstimate.pose.getX()
                  > Constants.FIELD_LENGTH + Constants.FIELD_BORDER_MARGIN
              || megatag2PoseEstimate.pose.getY() < -Constants.FIELD_BORDER_MARGIN
              || megatag2PoseEstimate.pose.getY()
                  > Constants.FIELD_WIDTH + Constants.FIELD_BORDER_MARGIN;
      if (cameraSeesTarget && !filterOut) {
        Optional<VisionFieldPoseEstimate> megatag2Estimate =
            processMegatag2PoseEstimate(megatag2PoseEstimate);

        if (megatag2Estimate.isPresent()) {
          if (isFrontRightLL) {
            Logger.recordOutput(
                "Vision/Front/" + "Megatag2Estimate",
                megatag2Estimate.get().getVisionRobotPoseMeters());
            RobotState.getInstance().setEstimatedPoseFrontRight(megatag2Estimate.get());
          } else if (megatag2PoseEstimate.isFrontLimelight) {
            Logger.recordOutput(
                "Vision/Front/" + "Megatag2Estimate",
                megatag2Estimate.get().getVisionRobotPoseMeters());
            RobotState.getInstance().setEstimatedPoseFrontLeft(megatag2Estimate.get());
          } else {
            Logger.recordOutput(
                "Vision/Back/" + "Megatag2Estimate",
                megatag2Estimate.get().getVisionRobotPoseMeters());
            RobotState.getInstance().setEstimatedPoseBack(megatag2Estimate.get());
          }
        }
      }
    }
  }

  private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
      MegatagPoseEstimate poseEstimate) {
    Pose2d loggedRobotPose = RobotState.getInstance().getOdometryPose();
    Pose2d measuredPose = poseEstimate.pose;
    if (poseEstimate.avgTagDist > 3) {
      System.out.println("Returning optional.empty");
      return Optional.empty();
    }

    double poseDelta = measuredPose.getTranslation().getDistance(loggedRobotPose.getTranslation());

    // TODO: Tag filtering?

    double xyStdDev;
    // if (poseEstimate.fiducialIds.length > 0) {
    // multiple targets detected
    if (poseEstimate.fiducialIds.length >= 2 && poseEstimate.avgTagArea > 0.1) {

      xyStdDev = 0.2;
    }
    // we detect at least one of our speaker tags and we're close to it.
    else if (
    /* TODO: doesSeeReefTag() && */ poseEstimate.avgTagArea > 0.2) {

      xyStdDev = 0.5;
    }
    // 1 target with large area and close to estimated pose
    else if (poseEstimate.avgTagArea > 0.8 && poseDelta < 0.5) {
      xyStdDev = 0.5;

    }
    // 1 target farther away and estimated pose is close
    else if (poseEstimate.avgTagArea > 0.1 && poseDelta < 0.3) {

      xyStdDev = 1.0;
    } else if (poseEstimate.fiducialIds.length > 1) {

      xyStdDev = 1.2;
    } else {

      xyStdDev = 2.4;
    }

    Logger.recordOutput("Vision/Front/" + "Megatag2StdDev", xyStdDev);
    Logger.recordOutput("Vision/Front/" + "Megatag2AvgTagArea", poseEstimate.avgTagArea);
    Logger.recordOutput("Vision/Front/" + "Megatag2PoseDifference", poseDelta);

    Matrix<N3, N1> visionMeasurementStdDevs =
        VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(3600));
    measuredPose = new Pose2d(measuredPose.getTranslation(), loggedRobotPose.getRotation());

    return Optional.of(
        new VisionFieldPoseEstimate(
            measuredPose, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
    // }

    // System.out.println("Returning optional.empty");
    // return Optional.empty();
  }

  public MegatagPoseEstimate getBotPose2dBlue() {
    if (inputs.megatag2PoseEstimateFrontLeft == null && inputs.megatag2PoseEstimateBack == null) {
      return null;
    }
    if (inputs.megatag2PoseEstimateFrontLeft == null) {
      return inputs.megatag2PoseEstimateBack;
    }
    if (inputs.megatag2PoseEstimateBack == null) {
      return inputs.megatag2PoseEstimateFrontLeft;
    }
    if (inputs.lowestTagAmbiguityFrontLeft < inputs.lowestTagAmbiguityBack) {
      return inputs.megatag2PoseEstimateFrontLeft;
    } else {
      return inputs.megatag2PoseEstimateBack;
    }
  }

  public double getLatencySecondsFrontLeft() {
    return inputs.frontLeftAprilTagCaptureLatencySeconds
        + inputs.frontLeftAprilTagPipelineLatencySeconds;
  }

  public double getLatencySecondsBack() {
    return inputs.backAprilTagCaptureLatencySeconds + inputs.backAprilTagPipelineLatencySeconds;
  }

  public double getLatencySecondsFrontRight() {
    return inputs.frontRightAprilTagCaptureLatencySeconds
        + inputs.frontRightAprilTagPipelineLatencySeconds;
  }

  public double getLowestTagAmbiguityFrontLeft() {
    return inputs.lowestTagAmbiguityFrontLeft;
  }

  public double getLowestTagAmbiguityBack() {
    return inputs.lowestTagAmbiguityBack;
  }

  public double getLowestTagAmbiguityFrontRight() {
    return inputs.lowestTagAmbiguityFrontRight;
  }

  public int getNumberFiducialsSpottedFrontLeft() {
    return inputs.frontLeftFiducials.length;
  }

  public int getNumberFiducialsSpottedBack() {
    return inputs.backFiducials.length;
  }

  public int getNumberFiducialsSpottedFrontRight() {
    return inputs.frontRightFiducials.length;
  }
}
