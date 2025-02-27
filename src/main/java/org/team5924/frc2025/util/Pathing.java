/*
 * Pathing.java
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

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.ArrayList;
import java.util.List;
import org.team5924.frc2025.Constants;

/** Add your docs here. */
public class Pathing {
  public static Pose2d getClosestPose(Pose2d currentPose, boolean isLeftTarget) {
    if (isLeftTarget) {
      ArrayList<Pose2d> leftBranchPoses = new ArrayList<>();
      for (var posePair : Constants.Reef.branchLeft2d) {
        leftBranchPoses.add(posePair.get(1));
        // Get 1 because it's the "shooting" pose
      }

      return currentPose.nearest(leftBranchPoses);
    } else {
      ArrayList<Pose2d> rightBranchPoses = new ArrayList<>();
      for (var posePair : Constants.Reef.branchRight2d) {
        rightBranchPoses.add(posePair.get(1));
        // Get 1 because it's the "shooting" pose
      }

      return currentPose.nearest(rightBranchPoses);
    }
  }

  // creates a path with a single waypoint which is the destination
  public static PathPlannerPath createPath(Pose2d currentPose, Pose2d destinationPose2d) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            currentPose,
            destinationPose2d.transformBy(
                new Transform2d(
                    0.5 * Math.cos(destinationPose2d.getRotation().getRadians()),
                    0.5 * Math.sin(destinationPose2d.getRotation().getRadians()),
                    new Rotation2d())),
            destinationPose2d);
    List<RotationTarget> holonomicRotations = new ArrayList<>();
    holonomicRotations.add(new RotationTarget(0.75, destinationPose2d.getRotation()));

    return new PathPlannerPath(
        waypoints,
        holonomicRotations,
        new ArrayList<>(),
        new ArrayList<>(),
        new ArrayList<>(),
        new PathConstraints(1.5, 1, 180, 180), // insert pathconstraints here
        null, // null for on-the-fly path
        new GoalEndState(0.0, destinationPose2d.getRotation()),
        false);
  }
}
