// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

/** Add your docs here. */
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.kinematics.SwerveModuleState;

    public class CTREModuleState {

        /**
         * Miniminze the change in heading the desired swerve module state would require by potentially
         * reversing the direction the wheel spins. Customized from WPILib's version to include placing
         * in appropriate scope for CTRE onboard control.
         * 
         * @param desiredState The desired state.
         * @param curentAngle The current module angle.
         */
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90){
                targetSpeed = -targetSpeed;
                targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
            }
            return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
        }


        /**
         * @param scopeReferance Current Angle
         * @param newAngle Target Angle
         * @return Closes angle withen scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReferance, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = scopeReferance % 360;
            if (lowerOffset >= 0) {
                lowerBound = scopeReferance - lowerOffset;
                upperBound = scopeReferance - (360 + lowerOffset);
            } else {
                upperBound = scopeReferance - lowerOffset;
                lowerBound = scopeReferance - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle -= 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - scopeReferance > 180) {
                newAngle -= 360; 
            } else if (newAngle - scopeReferance < -180) {
                newAngle += 360;
            }
            return newAngle;
        }
    }
