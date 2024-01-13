// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier; 

/** An example command that uses an example subsystem. */
public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainsubsystem;

    private final DoubleSupplier m_traslationXSupplier;
    private final DoubleSupplier m_traslationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier m_traslationXSupplier,
                               DoubleSupplier m_traslationYSupplier,
                               DoubleSupplier m_rotationSupplier; ) {
            this.m_drivetrainsubsystem = drivetrainSubsystem;
            this.m_translationXSupplier = translationXSupplier;
            this.m_translationYSupplier = translationYSupplier;
            this.m_rotationSupplier = rotationSupplier;

            addRequirements(drivetrainSubsystem);
}

    @Override
    public void execute() {
      //You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
      m_drivetrainsubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                  m_traslationXSupplier.getAsDouble(),
                  m_traslationYSupplier.getAsDouble(),
                  m_rotationSupplier.getAsDouble(),
                  m_drivetrainsubsystem.getGyroscopeRotation()
            )
      );
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrainsubsystem.drive(new ChassisSpeeds(vxMetersPerSecond:0.0, vyMetersPerSecond:0.0, omegaRadiansPerSecond:0.0));
    }
}
