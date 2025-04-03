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
  private final TalonFX rotateTalon;

  private final StatusSignal<Angle> rotatePosition;
  private final StatusSignal<AngularVelocity> rotateVelocity;
  private final StatusSignal<Voltage> rotateAppliedVoltage;
  private final StatusSignal<Current> rotateSupplyCurrent;
  private final StatusSignal<Current> rotateTorqueCurrent;
  private final StatusSignal<Temperature> rotateTempCelsius;

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
    rotateTalon = new TalonFX(Constants.CLIMBER_CAN_ID, Constants.CLIMBER_BUS);

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = Constants.CLIMBER_INVERT;
    config.MotorOutput.NeutralMode = Constants.CLIMBER_NEUTRAL_MODE;
    config.CurrentLimits.SupplyCurrentLimit = Constants.CLIMBER_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotateTalon.getConfigurator().apply(config);

    // Get select status signals and set update frequency
    rotatePosition = rotateTalon.getPosition();
    rotateVelocity = rotateTalon.getVelocity();
    rotateAppliedVoltage = rotateTalon.getMotorVoltage();
    rotateSupplyCurrent = rotateTalon.getSupplyCurrent();
    rotateTorqueCurrent = rotateTalon.getTorqueCurrent();
    rotateTempCelsius = rotateTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rotatePosition,
        rotateVelocity,
        rotateAppliedVoltage,
        rotateSupplyCurrent,
        rotateTorqueCurrent,
        rotateTempCelsius);

    // Disables status signals not called for update above
    rotateTalon.optimizeBusUtilization(0, 1.0);
    rotateTalon.setPosition(0);
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

    inputs.rotateMotorConnected =
        BaseStatusSignal.refreshAll(
                rotatePosition,
                rotateVelocity,
                rotateAppliedVoltage,
                rotateSupplyCurrent,
                rotateTorqueCurrent,
                rotateTempCelsius)
            .isOK();
    inputs.rotatePositionRads =
        Units.rotationsToRadians(rotatePosition.getValueAsDouble()) / reduction;
    inputs.rotateVelocityRadsPerSec =
        Units.rotationsToRadians(rotateVelocity.getValueAsDouble()) / reduction;
    inputs.rotateAppliedVoltage = rotateAppliedVoltage.getValueAsDouble();
    inputs.rotateSupplyCurrentAmps = rotateSupplyCurrent.getValueAsDouble();
    inputs.rotateTorqueCurrentAmps = rotateTorqueCurrent.getValueAsDouble();
    inputs.rotateTempCelsius = rotateTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    rotateTalon.setControl(voltageOut.withOutput(volts));
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
    rotateTalon.setControl(positionOut.withPosition(Radians.of(rads)));
  }
}
