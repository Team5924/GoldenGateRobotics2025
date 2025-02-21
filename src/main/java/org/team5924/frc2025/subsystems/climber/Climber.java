/*
 * Climber.java
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

package org.team5924.frc2025.subsystems.climber;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.RobotState;
import org.team5924.frc2025.subsystems.elevator.Elevator.ElevatorState;
import org.team5924.frc2025.subsystems.pivot.AlgaePivot.AlgaePivotState;
import org.team5924.frc2025.util.LoggedTunableNumber;

@Setter
@Getter
public class Climber extends SubsystemBase {
  public enum ClimberState {
    // Pulling onto cage, lifting robot
    CLIMB(new LoggedTunableNumber("Climber/ClimbingVoltage", 12.0)),

    // Default state, will be here most of the match
    STOW(new LoggedTunableNumber("Climber/StowVoltage", 0.0)),

    // Ready to climb
    READY_TO_CLIMB(new LoggedTunableNumber("Climber/ReadyToClimbVoltage", 0.0)),

    // Lowering robot
    REVERSE_CLIMB(new LoggedTunableNumber("Climber/ReverseClimbingVoltage", -12.0));

    private final LoggedTunableNumber volts;

    /**
     * Initializes the ClimberState with a specified voltage configuration.
     *
     * @param volts the tunable voltage parameter defining the output level for this state
     */
    ClimberState(LoggedTunableNumber volts) {
      this.volts = volts;
    }
  }

  // 1 = up, -1 = down
  // private double voltageMultiplier = 1;

  private ClimberState goalState = ClimberState.STOW;
  private ClimberState lastState;

  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private static final LoggedTunableNumber laserCanDetectThreshold =
      new LoggedTunableNumber("Climber/LaserCAN/DetectThreshold", 20);

  /**
   * Constructs a new Climber instance.
   * 
   * Initializes the hardware interface using the provided ClimberIO, sets up a disconnection alert 
   * for monitoring the climber's connectivity, and starts the state timer to manage state transitions.
   *
   * @param io the ClimberIO instance used for communication with the climber hardware
   */
  public Climber(ClimberIO io) {
    this.io = io;

    disconnected = new Alert("Climber disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  /**
   * Processes periodic updates for the climber subsystem.
   *
   * <p>Updates sensor inputs and logs data, sets the disconnection alert based on motor connectivity,
   * and transitions the climber to READY_TO_CLIMB when in STOW state with appropriate sensor and state conditions.
   * When the goal state changes, the state timer is reset to measure elapsed time for the new state.
   * Finally, commands the climber motor with the voltage defined by the current goal state and logs the current target.
   *
   * @implNote Designed to be called periodically by the subsystem scheduler.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    disconnected.set(!inputs.motorConnected);

    // If the robot's state is STOW && the cage is within range && elevator + algae pivot are both
    // stow,
    // then set the robot's state to READY_TO_CLIMB
    if (getGoalState() == ClimberState.STOW
        && isCageInClimber()
        && RobotState.getInstance().getElevatorState() == ElevatorState.INTAKE
        && RobotState.getInstance().getAlgaePivotState() == AlgaePivotState.INTAKE_FLOOR) {
      setGoalState(ClimberState.READY_TO_CLIMB);
    }

    if (getGoalState() != lastState) {
      stateTimer.reset();
      lastState = getGoalState();
    }

    io.runVolts(goalState.volts.getAsDouble());
    Logger.recordOutput("Climber/Climber Goal", goalState.toString());
  }

  /**
   * Updates the climber's target operational state and synchronizes this change with the global robot state.
   *
   * <p>Assigns the specified state to the climber's internal goal and notifies the RobotState subsystem accordingly.
   * Supported states include:
   * <ul>
   *   <li>{@code CLIMB} - Initiates the climbing operation.</li>
   *   <li>{@code STOW} - Retracts or deactivates the climbing mechanism.</li>
   *   <li>{@code READY_TO_CLIMB} - Prepares the system for climbing.</li>
   *   <li>{@code REVERSE_CLIMB} - Engages reverse climbing actions.</li>
   * </ul>
   *
   * <p>Note: Transition validation for disallowed state changes (e.g., directly from STOW to CLIMB or REVERSE_CLIMB)
   * is indicated in commented-out code. No exceptions are thrown if an unsupported transition is attempted.
   *
   * @param goalState the desired target state for the climber
   */
  public void setGoalState(ClimberState goalState) {
    // Validate state transitions
    // if (getGoalState() == ClimberState.STOW
    //     && (goalState == ClimberState.CLIMB || goalState == ClimberState.REVERSE_CLIMB)) {
    //   DriverStation.reportError(
    //       "Cannot transition Climber from STOW to "
    //           + goalState.name()
    //           + ".  Robot needs to be READY_TO_CLIMB before performing any climbing action",
    //       new StackTraceElement[] {
    //         new StackTraceElement("Climber", "setGoalState", "Climber", 106)
    //       });
    //   return;
    // }

    this.goalState = goalState;
    switch (goalState) {
      case CLIMB -> RobotState.getInstance().setClimberState(ClimberState.CLIMB);
      case STOW -> RobotState.getInstance().setClimberState(ClimberState.STOW);
      case READY_TO_CLIMB -> RobotState.getInstance().setClimberState(ClimberState.READY_TO_CLIMB);
      case REVERSE_CLIMB -> RobotState.getInstance().setClimberState(ClimberState.REVERSE_CLIMB);
    }
  }

  /**
   * Sets the goal state of the climber when it isn't moving (neither Dpad Up nor Dpad Down is
   * pressed).
   */
  // public void setGoalStateToNotMoving() {
  //   switch (getGoalState()) {
  //     case MOVING ->
  //         setGoalState(ClimberState.CLIMB); // the climber stopped moving, return to climb state
  //     default -> {} // the climber was not moving in the first place, don't need a state change
  // here
  //   }
  // }

  /**
   * Checks if the climber's LaserCAN sensor detects the presence of the cage.
   *
   * Detection is confirmed when the LaserCAN sensor returns a valid measurement status 
   * and the measured distance is less than the configured threshold.
   *
   * @return true if a valid measurement is obtained and the distance is below the threshold, false otherwise.
   */
  public boolean isCageInClimber() {
    return inputs.laserCanMeasurement.getStatus() == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && inputs.laserCanMeasurement.getDistance()
            < (int) Math.floor(laserCanDetectThreshold.get());
  }

  /**
   * Applies a specific voltage to the climber motor by relaying the command to the I/O interface.
   *
   * <p>The voltage provided is directly forwarded to the climber system's motor controller to control
   * the mechanism's power output.
   *
   * @param volts the voltage to apply to the climber motor, in volts
   */
  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  /**
   * Adjusts the climber mechanism to the specified angle in radians.
   *
   * <p>This method delegates the command to the climber's IO interface to update the
   * current angle as measured in radians.
   *
   * @param rads the desired angle in radians
   */
  public void setAngle(double rads) {
    io.setAngle(rads);
  }
}
