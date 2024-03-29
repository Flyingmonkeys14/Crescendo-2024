// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimebase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import frc.robot.Constants;

/**
 * CTRE config constants =
 */
public final class CTREConfigs {
    public TalonFXConfigurator swerveAngleConfig;
    public TalonFXConfigurator swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    /**
     * CTRE config constants
     */
    public CTREConfigs() {
        swerveAngleConfig = new TalonFXConfigurator();
        swerveDriveFXConfig = new TalonFXConfigurator();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, Constants.Swerve.angleContinuousCurrentLimit,
            Constants.Swerve.anglePeakCurrentLimit, Constants.Swerve.anglePeakCurrentDuration);

    swerveAngleFXConfig.slot0.kP = Constants.Swerve.driveKP;
    swerveAngleFXConfig.slot0.kI = Constants.Swerve.driveKI;
    swerveAngleFXConfig.slot0.kD = Constants.Swerve.driveKD;
    swerveAngleFXConfig.slot0.kF = Constants.Swerve.driveKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
    Constants.Swerve.driveEnableCurrentLimit, Constants.Swerve.driveContinuousCurrentLimit,
    Constants.Swerve.drivePeakCurrentLimit, Constants.Swerve.drivePeakCurrentDuration);

    swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
    swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
    swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
    swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
    swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;

/* Swerve CANCoder Configuration */
swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
swerveCanCoderConfig.initializationStrategy =
SensorInitializationStrategy.BootToAbsolutePosition;
swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}