// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;
import edu.wpi.first.math.controller.SimpleMotorFeedForward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.Robot;
import frc.robot.modules.CTREModuleState;
import frc.robot.modules.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.Talon;
import com.ctre.phoenix.sensors.CANcoder;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
public int moduleNumber;
private Rotation2d angleOffset;
private Rotation2d lastAngle;
private double angleDiagnostic = 0;

private Talon mAngleMotor;
private Talon mDriveMotor;
private CANcoder angleEncoder;

SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA); 

public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Drive Motor Config */
mAngleMotor = new Talon(moduleConstants.angleMotorID);

/* Drive Motor Config */
mDriveMotor = new Talon(moduleConstants.driveMotorID);
configDriveMotor();

lastAngle = getState().angle;
}

public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
/* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
desiredState = CTREModuleState.optimize(desiredState, getState().angle);
setAngle(desiredState);
setSpeed(desiredState, isOpenLoop);
}

private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
else{
    double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
}
}

private void setAngle(SwerveModuleState desireState){
    Rotation2d angle = (Math.abs(desireState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desireState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

    mAngleMotor.set(ControlMode.Position,  Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
    lastAngle = angle;
    angleDiagnostic = angle.getDegrees();
}

private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
}

public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
}

public double getAngleDiagnostic(){
    return angleDiagnostic;
}

public void resetToAbsolute(){
    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
}

private void configAngleEncoder(){
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
}

private void configAngleMotor(){
    mAngleMotor.configFactoryDefault();
    mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    resetToAbsolute();
}

private void configDriveMotor(){
    mDriveMotor.configFactoryDefault();
    mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    mDriveMotor.setSelectedSensorPosition(0);
}

public SwerveModuleState getState(){
    return new SwerveModuleState(
        Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
        getAngle()
    );
}

public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
        Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
        getAngle()
    );
}
public double getDriveEncoder(){
    return mDriveMotor.getSelectedSensorPosition();
}
}