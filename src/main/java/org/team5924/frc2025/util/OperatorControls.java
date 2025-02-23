/*
 * OperatorControls.java
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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team5924.frc2025.Constants;

/** Interface for physical override switches on operator console. */
public class OperatorControls {
  private final GenericHID buttonBoard;

  public OperatorControls(int port) {
    buttonBoard = new GenericHID(port);
  }

  public Trigger getL1Button() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_L1));
  }

  public Trigger getL2Button() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_L2));
  }

  public Trigger getL3Button() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_L3));
  }

  public Trigger getL4Button() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_L4));
  }

  public Trigger getElevatorToIntakeButton() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_INTAKE));
  }

  public Trigger getAlgaeL2() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_ALGAE_L2));
  }

  public Trigger getAlgaeL3() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_ALGAE_L3));
  }

  public Trigger getAlgaeScore() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_ELEVATOR_ALGAE_SCORE));
  }

  public Trigger getUnusedButton1() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_UNUSED_1));
  }

  public Trigger getUnusedButton2() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_UNUSED_2));
  }

  public Trigger getUnusedButton3() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_UNUSED_3));
  }

  public Trigger getUnusedButton4() {
    return new Trigger(() -> buttonBoard.getRawButton(Constants.OP_UNUSED_4));
  }
}
