/*
 * VisionFieldPoseEstimate.java
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

package org.team5924.frc2025.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionFieldPoseEstimate {

  private final Pose2d visionRobotPoseMeters;
  private final double timestampSeconds;
  private final Matrix<N3, N1> visionMeasurementStdDevs;

  public VisionFieldPoseEstimate(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.visionRobotPoseMeters = visionRobotPoseMeters;
    this.timestampSeconds = timestampSeconds;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
  }

  public VisionFieldPoseEstimate() {
    this.visionRobotPoseMeters = new Pose2d();
    this.timestampSeconds = 0.0;
    this.visionMeasurementStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  }

  public Pose2d getVisionRobotPoseMeters() {
    return visionRobotPoseMeters;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return visionMeasurementStdDevs;
  }
}
