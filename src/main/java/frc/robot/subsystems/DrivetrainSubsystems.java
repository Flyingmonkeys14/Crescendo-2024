// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfiurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinmatics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
/**
 * The maximum voltage that will be delivered to the drive motors.
 * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
 */
  public static final double MAX_VOLTAGE = 12.0;
// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
// The formula for calculating the theoretical maximum velocity is:
//   <Motor free speed rpm>  /  60 * <Drive reduction>  *  <Wheel diameter meters>  * pi
//  By defaul this value is setup for a Mk3 standard module using 

/**
 * The maximum velocity of the robot in meters per second.
 * This is a measure of how fast the robot should be able to drive in a straight line
 * 
 */
public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
  // Front left
  new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
  // Front right
  new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
  // Back right
  new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
  // Back left
  new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
);

// These are our modules. We initialize them in the constricor.
private final SwerveModule m_frontLeftModule; 
private final SwerveModule m_frontRightModule;
private final SwerveModule m_backLeftModule;
private final SwerveModule m_backRightModule;

private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(vxMetersPerSecond:0.0, vyMetersPerSecond:0.0, omegaRadianPerSecond:0.0);

public DrivetrainSubsystem() {
  ShuffleboardTab tab = Suffleboard.getTab(title:"Drivetrain");

  //change to Kraken
  
  m_frontLeftModule - Mk4iSwerveModuleHelper.createFalcon500(
    tab.getLayout(title: "Front Left Module", BuiltInLayouts.kList)
    .withSize(width:2, height:4)
    .withPosition(columnindex:0, row Index:0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
    FRONT_LEFT_MODULE_DRIVE_MOTOR,
    FRONT_LEFT_MODULE_STEER_MOTOR,
    FRONT_LEFT_MODULE_STEER_ENCODER,
    FRONT_LEFT_MODULE_STEER_OFFSET);


//change to Kraken
    m_frontRightModule - Mk4iSwerveModuleHelper.createFalcon500(
    tab.getLayout(title: "Front Right Module", BuiltInLayouts.kList)
    .withSize(width:2, height:4)
    .withPosition(columnindex:0, row Index:0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
    FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    FRONT_RIGHT_MODULE_STEER_MOTOR,
    FRONT_RIGHT_MODULE_STEER_ENCODER,
    FRONT_RIGHT_MODULE_STEER_OFFSET);

     m_backLeftModule - Mk4iSwerveModuleHelper.createFalcon500(
    tab.getLayout(title: "Back Left Module", BuiltInLayouts.kList)
    .withSize(width:2, height:4)
    .withPosition(columnindex:0, row Index:0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
    BACK_LEFT_MODULE_DRIVE_MOTOR,
    BACK_LEFT_MODULE_STEER_MOTOR,
    BACK_LEFT_MODULE_STEER_ENCODER,
    BACK_LEFT_MODULE_STEER_OFFSET);

     m_backRightModule - Mk4iSwerveModuleHelper.createFalcon500(
    tab.getLayout(title: "Back Right Module", BuiltInLayouts.kList)
    .withSize(width:2, height:4)
    .withPosition(columnindex:0, row Index:0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
    BACK_RIGHT_MODULE_DRIVE_MOTOR,
    BACK_RIGHT_MODULE_STEER_MOTOR,
    BACK_RIGHT_MODULE_STEER_ENCODER,
    BACK_RIGHT_MODULE_STEER_OFFSET);

}

public void zeroGyroscope() {
  m_pigeon.setFusedHeading(0.0);
}

public Rotation2d getGyroscopRotation() {
  return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
}

@Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
  
}
