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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.team5924.frc2025.Constants;

/** Add your docs here. */
public class Pathing {
  public static List<Pose2d> getClosestPose(Pose2d currentPose, boolean isLeftTarget) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    List<List<Pose2d>> rightAllianceBranchPairs =
        (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? Constants.Reef.redBranchRight2d
            : Constants.Reef.blueBranchRight2d;
    List<List<Pose2d>> leftAllianceBranchPairs =
        (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? Constants.Reef.redBranchLeft2d
            : Constants.Reef.blueBranchLeft2d;

    if (isLeftTarget) {
      ArrayList<Pose2d> leftBranchPoses = new ArrayList<>();
      for (var posePair : leftAllianceBranchPairs) {
        leftBranchPoses.add(posePair.get(1));
        // Get 1 because it's the "shooting" pose
      }

      Pose2d nearestPose = currentPose.nearest(leftBranchPoses);
      return leftAllianceBranchPairs.get(leftBranchPoses.indexOf(nearestPose));
    } else {
      ArrayList<Pose2d> rightBranchPoses = new ArrayList<>();
      for (var posePair : rightAllianceBranchPairs) {
        rightBranchPoses.add(posePair.get(1));
        // Get 1 because it's the "shooting" pose
      }

      Pose2d nearestPose = currentPose.nearest(rightBranchPoses);
      return rightAllianceBranchPairs.get(rightBranchPoses.indexOf(nearestPose));
    }
  }

  // creates a path with a single waypoint which is the destination
  public static PathPlannerPath createPath(Pose2d currentPose, List<Pose2d> destinationPoses) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            currentPose, destinationPoses.get(0), destinationPoses.get(1));
    List<RotationTarget> holonomicRotations = new ArrayList<>();
    holonomicRotations.add(new RotationTarget(0.75, destinationPoses.get(1).getRotation()));

    return new PathPlannerPath(
        waypoints,
        holonomicRotations,
        new ArrayList<>(),
        new ArrayList<>(),
        new ArrayList<>(),
        new PathConstraints(1.5, 1, 180, 180), // insert pathconstraints here
        null, // null for on-the-fly path
        new GoalEndState(0.0, destinationPoses.get(1).getRotation()),
        false);
  }
}
