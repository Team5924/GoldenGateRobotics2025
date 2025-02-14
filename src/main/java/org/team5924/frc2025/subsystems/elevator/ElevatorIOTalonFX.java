/*
 * ElevatorIOTalonFX.java
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

package org.team5924.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import static org.team5924.frc2025.Constants.ELEVATOR_LEFT_INVERSION;
import org.team5924.frc2025.util.LoggedTunableNumber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANdi;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

/** TODO: Need to rezero elevator on min height. */

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  /* Motor Hardware */
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  /* Sensor Hardware */
  private final CoreCANdi elevatorCANdi;

  /* Configurators */
  private TalonFXConfigurator leaderTalonConfig;
  private TalonFXConfigurator followerTalonConfig;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs leaderMotorConfigs;
  private final MotorOutputConfigs followerMotorConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.5);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.12);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 5);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
  LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Elevator/MotionAcceleration", 200);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Elevator/MotionCruiseVelocity", 100);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("Elevator/MotionJerk", 1000);

  /* Status Signals */
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Temperature> rightTempCelsius;

  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final PositionVoltage positionControl =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(Constants.ELEVATOR_LEFT_TALON_ID);
    rightTalon = new TalonFX(Constants.ELEVATOR_RIGHT_TALON_ID);

    this.leaderTalonConfig = leftTalon.getConfigurator();
    this.followerTalonConfig = rightTalon.getConfigurator();

    // Constants used in CANdi construction
    final int kCANdiId = 39;
    final String kCANdiCANbus = "rio";

    // Construct the CANdi
    elevatorCANdi = new CANdi(kCANdiId, kCANdiCANbus);

    // Configure the CANdi for basic use
    CANdiConfiguration configs = new CANdiConfiguration();
    /** TODO: handle floating pin state with an error */
    configs.withDigitalInputs(
        new DigitalInputsConfigs()
            .withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow));

    // Write these configs to the CANdi
    StatusCode status = elevatorCANdi.getConfigurator().apply(configs);
    /** TODO: ALERT if config operation go bad. */
    Logger.recordOutput("Elevator/CANdiConfigApplyStatus", status);

    /* Motor Config Create */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 100;
    currentLimitsConfigs.StatorCurrentLimit = 100;

    leaderMotorConfigs = new MotorOutputConfigs();
    leaderMotorConfigs.Inverted = ELEVATOR_LEFT_INVERSION;
    leaderMotorConfigs.PeakForwardDutyCycle = 1.0;
    leaderMotorConfigs.PeakReverseDutyCycle = -1.0;
    leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    followerMotorConfigs = new MotorOutputConfigs();
    followerMotorConfigs.PeakForwardDutyCycle = 1.0;
    followerMotorConfigs.PeakReverseDutyCycle = -1.0;
    followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.MOTOR_TO_ELEVATOR_REDUCTION;

    /** TODO: check for bad config apply and error */
    leaderTalonConfig.apply(currentLimitsConfigs);
    leaderTalonConfig.apply(leaderMotorConfigs);
    leaderTalonConfig.apply(slot0Configs);
    leaderTalonConfig.apply(motionMagicConfigs);
    leaderTalonConfig.apply(openLoopRampsConfigs);
    leaderTalonConfig.apply(closedLoopRampsConfigs);

    followerTalonConfig.apply(currentLimitsConfigs);
    followerTalonConfig.apply(followerMotorConfigs);
    followerTalonConfig.apply(slot0Configs);
    followerTalonConfig.apply(motionMagicConfigs);
    followerTalonConfig.apply(openLoopRampsConfigs);
    followerTalonConfig.apply(closedLoopRampsConfigs);

    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTempCelsius);

    closedLoopReferenceSlope = leftTalon.getClosedLoopReferenceSlope();

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftTorqueCurrent,
                leftTempCelsius)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTorqueCurrent,
                rightTempCelsius)
            .isOK();

    inputs.leftPositionRads = leftPosition.getValue().in(Radians);
    inputs.leftVelocityRadsPerSec = leftVelocity.getValue().in(RadiansPerSecond);
    inputs.leftAppliedVolts = leftAppliedVolts.getValue().in(Volts);
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValue().in(Amps);
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValue().in(Amps);
    inputs.leftTempCelsius = leftTempCelsius.getValue().in(Celsius);

    inputs.rightPositionRads = rightPosition.getValue().in(Radians);
    inputs.rightVelocityRadsPerSec = rightVelocity.getValue().in(RadiansPerSecond);
    inputs.rightAppliedVolts = rightAppliedVolts.getValue().in(Volts);
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValue().in(Amps);
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValue().in(Amps);
    inputs.rightTempCelsius = rightTempCelsius.getValue().in(Celsius);

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    if (currentTime - prevReferenceSlopeTimestamp > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope)
              / (currentTime - prevReferenceSlopeTimestamp);
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.minSoftStop = elevatorCANdi.getS1Closed().getValue();
    inputs.maxSoftStop = elevatorCANdi.getS2Closed().getValue();

    updateTunableNumbers();
  }

  @Override
  public void updateTunableNumbers() {
    if (kA.hasChanged(hashCode())
        || kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionAcceleration.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      leaderTalonConfig.apply(slot0Configs);
      followerTalonConfig.apply(slot0Configs);
      leaderTalonConfig.apply(motionMagicConfigs);
      followerTalonConfig.apply(motionMagicConfigs);
    }
  }

  @Override
  public void setHeight(double heightMeters) {
    if (!DriverStation.isEnabled()) {
      leftTalon.setControl(new VoltageOut(0.0));
      return;
    }

    setpoint = heightMeters;
    leftTalon.setControl(new MotionMagicVoltage(metersToRotations(heightMeters)));
    Logger.recordOutput("Elevator/GoalHeight", heightMeters);
  }

  @Override
  public void setVoltage(double volts) {
    leftTalon.setControl(
        voltageControl
            .withOutput(volts)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue())
            .withLimitReverseMotion(elevatorCANdi.getS1Closed().getValue()));
  }

  @Override
  public void setPosition(double meters) {
    leftTalon.setControl(
        positionControl.withPosition(Radians.of(metersToRotations(meters) * 2 * Math.PI)));
  }

  public double rotationsToMeters(double rotations) {
    return rotations
        * 2
        * Math.PI
        * Constants.SPROCKET_RADIUS.in(Meters)
        / Constants.MOTOR_TO_ELEVATOR_REDUCTION;
  }

  public static double metersToRotations(double height) {
    return height
        * Constants.MOTOR_TO_ELEVATOR_REDUCTION
        / (2 * Math.PI * Constants.SPROCKET_RADIUS.in(Meters));
  }
}
