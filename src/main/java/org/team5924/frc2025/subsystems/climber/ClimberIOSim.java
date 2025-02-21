/*
 * ClimberIOSim.java
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2025.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim motorSim;
  private double appliedVoltage = 0.0;
  private static final DCMotor CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  /**
   * Constructs a ClimberIOSim and initializes its motor simulation.
   *
   * <p>Configures {@code motorSim} by creating a DC motor system model using
   * {@code LinearSystemId.createDCMotorSystem} with the {@code CLIMBER_GEARBOX} constant,
   * a reduction ratio defined in {@code Constants.CLIMBER_REDUCTION}, and a moment of inertia
   * specified by {@code Constants.CLIMBER_SIM_MOI}. This simulation setup emulates the climber's motor behavior.
   */
  public ClimberIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CLIMBER_GEARBOX, Constants.CLIMBER_REDUCTION, Constants.CLIMBER_SIM_MOI),
            CLIMBER_GEARBOX);
  }

  /**
   * Updates the climber simulation inputs based on current motor simulation readings.
   *
   * <p>If the driver station is disabled, the motor voltage is set to zero prior to simulation update.
   * The simulation is then advanced by the loop period defined in {@code Constants.LOOP_PERIODIC_SECONDS},
   * and the provided {@code ClimberIOInputs} object is populated with the updated angular position (in radians),
   * angular velocity (in radians per second), applied voltage, and the motor's supply current (in amps).
   *
   * @param inputs the {@link ClimberIOInputs} object to update with simulation state values
   */
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    motorSim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.positionRads = motorSim.getAngularPositionRad();
    inputs.velocityRadsPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
  }

  /**
   * Clamps and applies the specified voltage to the motor simulation.
   *
   * <p>The provided voltage is constrained to the range of -12.0V to 12.0V using a clamping function,
   * ensuring the applied voltage remains within safe operational limits. The clamped voltage is then set as
   * the input voltage for the motor simulation.
   *
   * @param volts the desired voltage for the motor, before being clamped
   */
  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVoltage);
  }

  /**
   * Halts the climber motor simulation by setting the applied voltage to 0.0 volts.
   *
   * <p>This method delegates to {@link #runVolts(double)} with a zero voltage parameter,
   * ensuring that the motor stops, which is particularly critical when the robot is disabled.</p>
   */
  @Override
  public void stop() {
    runVolts(0.0);
  }
}
