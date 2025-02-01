// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class VisionProfile {
        /* Limelight names */
        public static String frontLimelight = "limelight-front"; //TODO: Configure limelight name to match constant
        public static String rearLimelight = "limelight-rear"; //TODO: Configure limelight name to match contrant

        /* Calibrated limelight pipelines */
        public static int reefPipeline_Test = 0;
        public static int stationTestPipeline_Test = 0;
      
        /* Limelight Tx set point for right sided reef  */
        // Currently configured for test values, true set points are currently undetermined.
        public static double rightReefSetPoint_Tx = -2.0;
        public static double rightReefSetPointNegativeTolerance_Tx = rightReefSetPoint_Tx - 1;
        public static double rightReefSetPointPositiveTolerance_Tx = rightReefSetPoint_Tx + 1;
      
        /* Limelight Tx set point for left sided reef */
        // Currently configured for test values, true set points are currently undetermined.
        public static double leftReefSetPoint_Tx = 2.0;
        public static double leftReefSetPointNegativeTolerance_Tx = leftReefSetPoint_Tx - 1;
        public static double leftReefSetPointPositiveTolerance_Tx = leftReefSetPoint_Tx + 1;
      
        /* Limelight Tx set point for algae between reefs */
        // Currently configured for test values, true set points are currently undetermined.
        public static double centerReefSetPoint_Tx = 0.0;
        public static double centerReefSetPointNegativeTolerance_Tx = centerReefSetPoint_Tx - 1;
        public static double centerReefSetPointPositiveTolerance_Tx = centerReefSetPoint_Tx + 1;

        /* Limelight Ta set point for algae between reefs */
        // Currently configured for test values, true set points are currently undetermined.
        public static double centerReefSetPoint_Ta = 5.0;
        public static double centerReefSetPointNegativeTolerance_Ta = centerReefSetPoint_Ta - 1;
        public static double centerReefSetPointPositiveTolerance_Ta = centerReefSetPoint_Ta + 1;
      
        /* Limelight Tx set point for coral station */
        // Currently configured for test values, true set points are currently undetermined.
        public static double stationSetPoint_Tx = 0;
        public static double stationSetPointNegativeTolerance_Tx = stationSetPoint_Tx - 1;
        public static double stationSetPointPositiveTolerance_Tx = stationSetPoint_Tx + 1;
    }

    public static class SwerveProfile {
        /* Swerve PID gains */
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double defaultSwerveVelocity = 0.2;
    }
}
