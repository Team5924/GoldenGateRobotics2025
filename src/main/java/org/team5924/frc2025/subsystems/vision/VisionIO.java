/*
 * VisionIO.java
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

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.MegatagPoseEstimate;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean frontLeftLimelightSeesTarget = false;
    public boolean backLimelightSeesTarget = false;
    public boolean frontRightLimelightSeesTarget = false;

    public FiducialObservation[] frontLeftFiducials = new FiducialObservation[] {};
    public FiducialObservation[] backFiducials = new FiducialObservation[] {};
    public FiducialObservation[] frontRightFiducials = new FiducialObservation[] {};

    public MegatagPoseEstimate megatag2PoseEstimateFrontLeft = null;
    public MegatagPoseEstimate megatag2PoseEstimateBack = null;
    public MegatagPoseEstimate megatag2PoseEstimateFrontRight = null;

    public Pose2d megatag2PoseEstimateFrontLeftPose2d = null;
    public Pose2d megatag2PoseEstimateBackPose2d = null;
    public Pose2d megatag2PoseEstimateFrontRightPose2d = null;
    public int megatag2PoseEstimateFrontLeftTagCount = 0;
    public int megatag2PoseEstimateBackTagCount = 0;
    public int megatag2PoseEstimateFrontRightTagCount = 0;
    public double megatag2PoseEstimateFrontLeftAvgTagArea = 0;
    public double megatag2PoseEstimateBackAvgTagArea = 0;
    public double megatag2PoseEstimateFrontRightAvgTagArea = 0;

    public double lowestTagAmbiguityFrontLeft = 1;
    public double lowestTagAmbiguityBack = 1;
    public double lowestTagAmbiguityFrontRight = 1;

    public double frontLeftAprilTagPipelineLatencySeconds = 0.0;
    public double frontLeftAprilTagCaptureLatencySeconds = 0.0;

    public double backAprilTagPipelineLatencySeconds = 0.0;
    public double backAprilTagCaptureLatencySeconds = 0.0;

    public double frontRightAprilTagPipelineLatencySeconds = 0.0;
    public double frontRightAprilTagCaptureLatencySeconds = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
