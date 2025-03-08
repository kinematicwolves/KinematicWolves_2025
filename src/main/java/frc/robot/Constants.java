// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Constants {
    public class DriverProfile {
        /* Angular speed multiplier */
        public static final double kRotationMagnitude = 0.45;

        /* Driver station ports */
        public static final int driverPortNum = 0;
        public static final int operatorPortNum = 1;
        public static final int technicianPortNum = 2;

        /* Alignment Speed Multiplier */
        public static final double x_AlignmentMultiplier = 0.5;
        public static final double y_AlignmentMultiplier = 0.85; 

        /* Slow Mode Speeds */
        public static final double x_slowMode = 0.8;
        public static final double y_slowMode = 0.8;
        public static final double rx_slowMode = 0.4;
    }
    public class ElevatorProfile {
        /* CAN Id's */
        public static final int liftA_Id = 50;
        public static final int liftB_Id = 51;

        /* Motor Current Limit */
        public static final int currentLimit = 35;

        /* Elevator Height Limit */
        public static final double fwdSoftLimitNum = 300;

        /* Motor Idle Modes */
        public static final IdleMode defaultIdleMode = IdleMode.kBrake;

        /* PIDs */
        public static final double kP = 1.6;
        public static final double kI = 0;
        public static final double kD = 0.1;

        /* Encoder Error */
        public static final double encoderSetPointError = 1;

        /* Positions */
        public static final double homePos = -1;

        public static final double coralLvl1Pos = 35;
        public static final double coralLvl2Pos = 48.2;
        public static final double coralLvl3Pos = 111.6;
        public static final double coralLvl4Pos = 219;

        public static final double algaeLvl1Pos = 86;
        public static final double algaeLvl2Pos = 155;
        public static final double algaeScorePos = 31;

        public static final double shallowCagePos = 92;
    }

    public class WristProfile {
        /* CAN Id */
        public static final int motorID = 60;

        /* Motor Current Limit */
        public static final int currentLimit = 20;

        /* PIDs */
        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 2.8;

        /* Encoder Error */
        public static final double encoderSetPointError = 0.5;

        /* Positions */
        public static final double homePos = -10;
        public static final double algaeHomePos = 24;
        public static final double safeTravelPos = 10;

        public static final double coralLvl1Pos = 3;
        public static final double coralLvl2Pos = 7.4;
        public static final double coralLvl3Pos = 6.9;
        public static final double coralLvl4Pos = 10;

        public static final double algaeLvl1Pos = 47;
        public static final double algaeLvl2Pos = 43;
        public static final double algaeScorePos = 57.3;

        public static final double shallowCagePos = 0;
    }

    public class GripperProfile {
        /* CAN Id */
        public static final int motorId = 56;
        public static final int tofId = 57;

        /* Motor Current Limit */
        public static final double currentLimit = 20;

        /* Sensor Range Values */
        public static final double tofCoralRange = 75;

        /* Default Speeds (Percentage Outputs) */
        public static final double acquireCoralSpeed = 0.135;
        public static final double indexCoralSpeed = 0.09;

        public static final double intakeAlgaeSpeed = -0.3;
        public static final double holdAlgaeOutput = -0.3;
        public static final double outTakeSpeed = 0.3;
    }

    public class VisionProfile {
        /* Limelight names */
        public static String frontLimelight = "limelight-front";
        public static String rearLimelight = "limelight-rear";

        /* Calibrated limelight pipelines */
        public static int reefPipeline_Test = 0;
        public static int blueReefCenterPipeline = 1;
        public static int blueReefLeftPipeline = 2;
        public static int blueReefRightPipeline = 3;
        public static int redReefCenterPipeline = 4;
        public static int redReefLeftPipeline = 5;
        public static int redReedRightPipeline = 6;

        /* Proportional limits for front limelight */
        public static double frontProportionalTx = 7.5;
        public static double frontProportionalTy = 20;
        public static double frontProportionalTa = 20;
    }
}