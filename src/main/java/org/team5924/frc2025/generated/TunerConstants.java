/*
 * TunerConstants.java
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

package org.team5924.frc2025.generated;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import org.team5924.frc2025.util.swerve.ModuleLimits;

// import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(150)
          .withKI(0)
          .withKD(0.3)
          .withKS(0.0)
          .withKV(1.5)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(1.5).withKI(0).withKD(0).withKS(0.19169).withKV(0.65572);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Current kSlipCurrent = Amps.of(120.0);

  // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs =
      new Pigeon2Configuration()
          .withMountPose(
              new MountPoseConfigs()
                  .withMountPoseRoll(-0.08099313080310822)
                  .withMountPosePitch(-0.20943012833595276)
                  .withMountPoseYaw(91.5396957397461));

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus("Drive CANivore", "./logs/example.hoot");

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final LinearVelocity kSpeedAt12Volts = FeetPerSecond.of(18.47);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 4.909090909090909;

  private static final double kDriveGearRatio = 5.67;
  private static final double kSteerGearRatio = 12.1;
  private static final Distance kWheelRadius = Inches.of(1.875);

  private static final boolean kInvertLeftSide = true;
  private static final boolean kInvertRightSide = false;

  private static final int kPigeonId = 0;

  // These are only used for simulation
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 7;
  private static final int kFrontLeftSteerMotorId = 9;
  private static final int kFrontLeftEncoderId = 8;
  private static final Angle kFrontLeftEncoderOffset =
      Rotations.of(-0.062744140625 + 0.375); // account for calibrating at 45 degrees
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(12);
  private static final Distance kFrontLeftYPos = Inches.of(12);

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 6;
  private static final int kFrontRightEncoderId = 5;
  private static final Angle kFrontRightEncoderOffset =
      Rotations.of(0.088134765625 - 0.375); // account for calibrating at 45 degrees
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(12);
  private static final Distance kFrontRightYPos = Inches.of(-12);

  // Back Left
  private static final int kBackLeftDriveMotorId = 10;
  private static final int kBackLeftSteerMotorId = 12;
  private static final int kBackLeftEncoderId = 11;
  private static final Angle kBackLeftEncoderOffset =
      Rotations.of(-0.357177734375 - 0.375); // account for calibrating at 45 degrees
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-12);
  private static final Distance kBackLeftYPos = Inches.of(12);

  // Back Right
  private static final int kBackRightDriveMotorId = 1;
  private static final int kBackRightSteerMotorId = 3;
  private static final int kBackRightEncoderId = 2;
  private static final Angle kBackRightEncoderOffset =
      Rotations.of(0.079345703125 + 0.375); // account for calibrating at 45 degrees
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-12);
  private static final Distance kBackRightYPos = Inches.of(-12);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          Units.feetToMeters(18.47), // max speed, guess
          Units.feetToMeters(75.0), // max acceleration, guess
          Units.degreesToRadians(1080.0));

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  // public static CommandSwerveDrivetrain createDrivetrain() {
  //     return new CommandSwerveDrivetrain(
  //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
  //     );
  // }

  /**
   * s Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
   */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
