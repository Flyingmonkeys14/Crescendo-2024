// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class Conversions {
   /** mathematical conversions fo swerve calculations
    * */
   //change to kraken later
    public static double falconToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 2048.0));
    }


    /**
     * @param degrees degrees of rotation mechanism
     * @param gearRatio gear ratio between falcon and mechanism
     * @return falcon potition counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts falcon velocity counts
     * @param gearRatio gear ratio between falcon and mechanism (set to 1 for falcon rpm)
     * @return rpm of mechanism
     */
    public static double falconToRpm(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM; 
    }

    /**
     * @param RPM rpm of mechanism
     * @param gearRatio gear ratio between falcon and mechanism (set to 1 for falcon rpm) 
     * @return rpm of mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts falcon velocity counts
     * @param circumference circumference of wheel
     * @param gearRatio gear ratio between falcon and mechanism (set to 1 for falcon mps)
     * @return falcon velocity counts
     */

     public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRpm(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
     }

     /**
      * @param velocity velocity MPS
        @param circumference ircumference of wheel
        @param gearRatio gear ratio between falcon and mechanism (set to 1 for falcon MPS)
        @return falcon velocity counts
      */
      public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
      }

      /**
       * @param positionCounts falcon position counts
       * @param circumference circumference of wheel
       * @param gearRatio gear ratio between falcon and wheel
       * @return falcon position counts
       */
      public static double MetersToFalcon(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * 2048.0));
      }
    }
