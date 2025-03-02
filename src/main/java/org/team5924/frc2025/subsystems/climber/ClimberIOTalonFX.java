/*
 * ClimberIOTalonFX.java
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

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Temperature> rightTempCelsius;


  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  private final double reduction;

  // private static final LaserCan laserCan = new LaserCan(Constants.CLIMBER_LASER_CAN_ID);

  // private static final Alert laserCanDisconnectAlert =
  //     new Alert("Climber LaserCAN disconnected.", AlertType.kWarning);
  // private static final Alert laserCanInvalidMeasure =
  //     new Alert("Climber LaserCAN grabbed invalid measurement. See logs.", AlertType.kWarning);

  public ClimberIOTalonFX() {
    reduction = Constants.CLIMBER_REDUCTION;
    leftTalon = new TalonFX(Constants.LEFT_CLIMBER_CAN_ID, Constants.CLIMBER_BUS);
    rightTalon = new TalonFX(Constants.RIGHT_CLIMBER_CAN_ID, Constants. CLIMBER_BUS);

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = Constants.CLIMBER_INVERT;
    config.MotorOutput.NeutralMode = Constants.CLIMBER_NEUTRAL_MODE;
    config.CurrentLimits.SupplyCurrentLimit = Constants.CLIMBER_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftTalon.getConfigurator().apply(config);
    rightTalon.getConfigurator().apply(config);

    // Get select status signals and set update frequency
    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVoltage = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVoltage = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVoltage,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVoltage,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTempCelsius);

    // Disables status signals not called for update above
    leftTalon.optimizeBusUtilization(0, 1.0);
    rightTalon.optimizeBusUtilization(0, 1.0);

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // try {
    //   inputs.laserCanMeasurement = LaserCAN_Measurement.fromLaserCAN(laserCan.getMeasurement());
    //   inputs.laserCanConnected = true;
    //   laserCanDisconnectAlert.set(false);
    //   laserCanInvalidMeasure.set(false);
    // } catch (SensorRuntimeException e) {
    //   switch (e.getErrorType()) {
    //     case DISCONNECTED -> {
    //       inputs.laserCanConnected = false;
    //       laserCanDisconnectAlert.set(true);
    //     }
    //     case INVALID_DATA -> laserCanInvalidMeasure.set(true);
    //     default -> {
    //       if (Constants.ALLOW_ASSERTS) throw e;
    //       else System.err.println("FIX NOW: Unhandled SensorRuntimeException: " +
    // e.getMessage());
    //     }
    //   }
    // }

    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftTorqueCurrent,
                leftTempCelsius)
            .isOK();
    inputs.leftPositionRads =
        Units.rotationsToRadians(leftPosition.getValueAsDouble()) / reduction;
    inputs.leftVelocityRadsPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / reduction;
    inputs.leftAppliedVoltage = leftAppliedVoltage.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightTorqueCurrent,
                rightTempCelsius)
            .isOK();
    inputs.rightPositionRads =
        Units.rotationsToRadians(rightPosition.getValueAsDouble()) / reduction;
    inputs.rightVelocityRadsPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble()) / reduction;
    inputs.rightAppliedVoltage = rightAppliedVoltage.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    leftTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setAngle(double rads) throws IllegalArgumentException {
    if (rads < Constants.CLIMBER_MIN_RADS || rads > Constants.CLIMBER_MAX_RADS) {
      String message =
          "Cannot set climber angle to "
              + rads
              + " rads.  This value extends past the climber angle boundary.";
      Logger.recordOutput("Climber/InvalidAngle", message);
      throw new IllegalArgumentException(message);
    }
    rads = Math.max(Constants.CLIMBER_MIN_RADS, Math.min(rads, Constants.CLIMBER_MAX_RADS));
    leftTalon.setControl(positionOut.withPosition(Radians.of(rads)));
  }
}
