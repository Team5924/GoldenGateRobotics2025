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
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2025.util.FiducialObservation;
import org.team5924.frc2025.util.MegatagPoseEstimate;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean frontUpLimelightSeesTarget = false;
    public boolean backLimelightSeesTarget = false;
    public boolean frontDownLimelightSeesTarget = false;

    public FiducialObservation[] frontUpFiducials = new FiducialObservation[] {};
    public FiducialObservation[] backFiducials = new FiducialObservation[] {};
    public FiducialObservation[] frontDownFiducials = new FiducialObservation[] {};

    public MegatagPoseEstimate megatag2PoseEstimateFrontUp = null;
    public MegatagPoseEstimate megatag2PoseEstimateBack = null;
    public MegatagPoseEstimate megatag2PoseEstimateFrontDown = null;


    public Pose2d megatag2PoseEstimateFrontUpPose2d = null;
    public Pose2d megatag2PoseEstimateBackPose2d = null;
    public Pose2d megatag2PoseEstimateFrontDownPose2d = null;
    public int megatag2PoseEstimateFrontUpTagCount = 0;
    public int megatag2PoseEstimateBackTagCount = 0;
    public int megatag2PoseEstimateFrontDownTagCount = 0;
    public double megatag2PoseEstimateFrontUpAvgTagArea = 0;
    public double megatag2PoseEstimateBackAvgTagArea = 0;
    public double megatag2PoseEstimateFrontDownAvgTagArea = 0;

    public double lowestTagAmbiguityFrontUp = 1;
    public double lowestTagAmbiguityBack = 1;
    public double lowestTagAmbiguityFrontDown = 1;

    public double frontUpAprilTagPipelineLatencySeconds = 0.0;
    public double frontUpAprilTagCaptureLatencySeconds = 0.0;


    public double backAprilTagPipelineLatencySeconds = 0.0;
    public double backAprilTagCaptureLatencySeconds = 0.0;

    public double frontDownAprilTagPipelineLatencySeconds = 0.0;
    public double frontDownAprilTagCaptureLatencySeconds = 0.0;


    public Pose3d frontUpAprilTagDistance = new Pose3d();
    public Pose3d backAprilTagDistance = new Pose3d();
    public Pose3d frontDownAprilTagDistance = new Pose3d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
