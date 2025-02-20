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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.team5924.frc2025.Constants.ELEVATOR_LEFT_INVERSION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANdi;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2025.Constants;
import org.team5924.frc2025.util.LoggedTunableNumber;

/** TODO: Need to rezero elevator on min height. */

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  /* Motor Hardware */
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  /* Sensor Hardware */
  private final CoreCANdi elevatorCANdi;
  private final CANcoder elevatorCANCoder;

  /* Configurators */
  private TalonFXConfigurator leaderTalonConfig;
  private TalonFXConfigurator followerTalonConfig;
  private CANcoderConfigurator elevatorCANCoderConfig;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs leaderMotorConfigs;
  private final MotorOutputConfigs followerMotorConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;
  private final CANcoderConfiguration canCoderConfig;

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.00);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.13);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.4);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 7);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.07);
  LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.33);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Elevator/MotionAcceleration", 400);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Elevator/MotionCruiseVelocity", 400);
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

  private final VoltageOut voltageControl;
  private final PositionVoltage positionControl;
  private final MotionMagicVoltage magicMotionVoltage;

  /* Alerts */
  private final Alert updateMotorConfigAlert =
      new Alert("Update elevator motor config error!", Alert.AlertType.kWarning);

  private final Alert initalMotorConfigAlert =
      new Alert(
          "Initial elevator motor config error! Restart robot code to clear.",
          Alert.AlertType.kError);

  private final Alert candiPin1FloatAlert =
      new Alert("Elevator CANdiPin1 is floating. Check connection.", Alert.AlertType.kWarning);

  private final Alert candiPin2FloatAlert =
      new Alert("Elevator CANdiPin2 is floating. Check connection.", Alert.AlertType.kWarning);

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(Constants.ELEVATOR_LEFT_TALON_ID);
    rightTalon = new TalonFX(Constants.ELEVATOR_RIGHT_TALON_ID);
    elevatorCANCoder = new CANcoder(Constants.ELEVATOR_CANCODER_ID);

    this.leaderTalonConfig = leftTalon.getConfigurator();
    this.followerTalonConfig = rightTalon.getConfigurator();
    this.elevatorCANCoderConfig = elevatorCANCoder.getConfigurator();

    elevatorCANdi = new CANdi(Constants.ELEVATOR_CANDI_ID, Constants.ELEVATOR_CANDI_BUS);

    // Configure the CANdi for basic use
    CANdiConfiguration configs = new CANdiConfiguration();
    configs.withDigitalInputs(
        new DigitalInputsConfigs()
            .withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow));

    /* Motor Config Create */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 80;
    currentLimitsConfigs.StatorCurrentLimit = 80;

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
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // feedbackConfigs.FeedbackRemoteSensorID = Constants.ELEVATOR_CANCODER_ID;
    // feedbackConfigs.SensorToMechanismRatio = Constants.CANCODER_TO_ELEVATOR_REDUCTION;
    feedbackConfigs.RotorToSensorRatio = Constants.MOTOR_TO_ELEVATOR_REDUCTION;

    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = Constants.ELEVATOR_CANCODER_OFFSET;

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[15];

    statusArray[0] = leaderTalonConfig.apply(currentLimitsConfigs);
    statusArray[1] = leaderTalonConfig.apply(leaderMotorConfigs);
    statusArray[2] = leaderTalonConfig.apply(slot0Configs);
    statusArray[3] = leaderTalonConfig.apply(motionMagicConfigs);
    statusArray[4] = leaderTalonConfig.apply(openLoopRampsConfigs);
    statusArray[5] = leaderTalonConfig.apply(closedLoopRampsConfigs);
    statusArray[6] = leaderTalonConfig.apply(feedbackConfigs);

    statusArray[7] = followerTalonConfig.apply(currentLimitsConfigs);
    statusArray[8] = followerTalonConfig.apply(followerMotorConfigs);
    statusArray[9] = followerTalonConfig.apply(slot0Configs);
    statusArray[10] = followerTalonConfig.apply(motionMagicConfigs);
    statusArray[11] = followerTalonConfig.apply(openLoopRampsConfigs);
    statusArray[12] = followerTalonConfig.apply(closedLoopRampsConfigs);

    statusArray[13] = elevatorCANCoderConfig.apply(canCoderConfig);

    statusArray[14] = elevatorCANdi.getConfigurator().apply(configs);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
    initalMotorConfigAlert.set(isErrorPresent);
    Logger.recordOutput("Elevator/InitConfReport", statusArray);

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

    closedLoopReferenceSlope = leftTalon.getClosedLoopReferenceSlope();

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
        rightTempCelsius,
        closedLoopReferenceSlope);

    voltageControl =
        new VoltageOut(0)
            .withUpdateFreqHz(0.0)
            .withEnableFOC(true)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue())
            .withLimitReverseMotion(elevatorCANdi.getS1Closed().getValue());
    positionControl =
        new PositionVoltage(0)
            .withUpdateFreqHz(0.0)
            .withEnableFOC(true)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue())
            .withLimitReverseMotion(elevatorCANdi.getS1Closed().getValue());
    magicMotionVoltage =
        new MotionMagicVoltage(0)
            .withEnableFOC(true)
            .withLimitForwardMotion(elevatorCANdi.getS2Closed().getValue())
            .withLimitReverseMotion(elevatorCANdi.getS1Closed().getValue());

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));
    leftTalon.setPosition(0.0);
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
                leftTempCelsius,
                closedLoopReferenceSlope)
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

    inputs.posMeters = getHeight();
    inputs.velMetersPerSecond = getVelocity();

    inputs.motionMagicVelocityTarget =
        rotationsToMeters(leftTalon.getClosedLoopReferenceSlope().getValue());
    inputs.motionMagicPositionTarget =
        rotationsToMeters(leftTalon.getClosedLoopReference().getValue());

    inputs.setpointMeters = setpoint;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.minSoftStop = elevatorCANdi.getS1Closed().getValue();
    inputs.maxSoftStop = elevatorCANdi.getS2Closed().getValue();
  }

  @Override
  public void periodicUpdates() {
    updateTunableNumbers();
    isAtZero();

    candiPin1FloatAlert.set(elevatorCANdi.getS1State().getValue() == S1StateValue.Floating);
    candiPin2FloatAlert.set(elevatorCANdi.getS2State().getValue() == S2StateValue.Floating);
  }

  public void updateTunableNumbers() {
    if (kA.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || motionAcceleration.hasChanged(hashCode())
        || motionCruiseVelocity.hasChanged(hashCode())) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      slot0Configs.kG = kG.get();

      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      StatusCode[] statusArray = new StatusCode[4];

      statusArray[0] = leaderTalonConfig.apply(slot0Configs);
      statusArray[1] = followerTalonConfig.apply(slot0Configs);
      statusArray[2] = leaderTalonConfig.apply(motionMagicConfigs);
      statusArray[3] = followerTalonConfig.apply(motionMagicConfigs);

      boolean isErrorPresent = false;
      for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
      updateMotorConfigAlert.set(isErrorPresent);
      Logger.recordOutput("Elevator/Leader/UpdateConfReport", statusArray);
    }
  }

  @Override
  public void setHeight(double heightMeters) {
    if (!DriverStation.isEnabled()) {
      leftTalon.setControl(new VoltageOut(0.0));
      return;
    }

    setpoint = heightMeters;
    leftTalon.setControl(magicMotionVoltage.withPosition(metersToRotations(heightMeters)));
    Logger.recordOutput("Elevator/GoalHeight", heightMeters);
  }

  @Override
  public void setVoltage(double volts) {
    Logger.recordOutput("Elevator/JoystickValue", volts);
    leftTalon.setControl(voltageControl.withOutput(volts));
  }

  public boolean isAtZero() {
    if (elevatorCANdi.getS1Closed().getValue()) {
      leftTalon.setPosition(0.0);
      return true;
    }

    return false;
  }

  public double rotationsToMeters(double rotations) {
    return (rotations
            // * 2 // Multiply by 2 to account for cascade rigging
            * 2
            * Math.PI
            * Constants.SPROCKET_RADIUS.in(Meters)
            / Constants.MOTOR_TO_ELEVATOR_REDUCTION)
        * 2;
  }

  public static double metersToRotations(double height) {
    return height
        // * 2 // Multiply by 2 to account for cascade rigging
        * Constants.MOTOR_TO_ELEVATOR_REDUCTION
        / (2 * Math.PI * Constants.SPROCKET_RADIUS.in(Meters))
        / 2;
  }

  private double getHeight() {
    return rotationsToMeters(leftTalon.getPosition().getValueAsDouble());
  }

  private double getVelocity() {
    return rotationsToMeters(leftTalon.getVelocity().getValueAsDouble());
  }
}
