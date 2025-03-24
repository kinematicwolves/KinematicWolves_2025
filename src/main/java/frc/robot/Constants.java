// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Constants {
    public class DriverProfile {
        /* Angular speed multiplier */
        public static final double kRotationMagnitude = 0.48;

        /* Slow mode speeds */
        public static final double x_SlowMultiplier = 0.5;
        public static final double y_SlowMultiplier = 0.5;
        public static final double rx_SlowMultiplier = 0.5;

        /* Driver station ports */
        public static final int driverPortNum = 0;
        public static final int operatorPortNum = 1;
        public static final int technicianPortNum = 2;

        /* Alignment Speed Multiplier */
        public static final double x_AlignmentMultiplier = 0.5;
        public static final double y_AlignmentMultiplier = 0.85; 

        /* Alignment Speeds */
        public static final double x_slowMode = 0.7;
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
        public static final double kP = 0.06;
        public static final double kI = 0.00001;
        public static final double kD = 0.5;

        /* Encoder Error */
        public static final double encoderSetPointError = 2;

        /* Positions */
        public static final double homePos = -1;

        public static final double coralLvl1Pos = 14.6;
        public static final double coralLvl2Pos = 19.4;
        public static final double coralLvl3Pos = 37.2;
        public static final double coralLvl4Pos = 74.8;

        public static final double algaeLvl1Pos  = 35;
        public static final double algaeLvl2Pos  = 47;
        public static final double algaeScorePos = 31  * 20/45;

        public static final double maxElevatorCG = 20;

        // public static final double shallowCagePos = 92 * 20/45;
    }

    public class WristProfile {
        /* CAN Id */
        public static final int motorID = 60;

        /* Motor Current Limit */
        public static final int currentLimit = 20;

        /* PIDs */
        public static final double kP = 0.06;
        public static final double kI = 0;
        public static final double kD = 2.8;

        /* Encoder Error */
        public static final double encoderSetPointError = 0.5;

        /* Positions */
        public static final double homePos = -0.5;
        public static final double algaeHomePos = 24;
        public static final double safeTravelPos = 10;

        public static final double coralLvl1Pos = 3;
        public static final double coralLvl2Pos = 7.4;
        public static final double coralLvl3Pos = 7.1;
        public static final double coralLvl4Pos = 12;

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
        public static final double acquireCoralSpeed = 0.145;
        public static final double indexCoralSpeed = 0.09;

        public static final double intakeAlgaeSpeed = -0.3;
        public static final double holdAlgaeOutput = -0.3;
        public static final double outTakeSpeed = 0.3;
    }

    public class VisionProfile {
        /* Limelight names */
        public static String frontLimelight = "limelight-front";
        public static String elevatorLimelight = "limelight-elevator";

        /* Calibrated frontLimelight Pipeline */
        public static int autoPipeline = 0;

        /* Calibrated elevator limelight pipelines */
        public static int reefPipeline_Test = 0;
        public static int blueReefCenterPipeline = 1;
        public static int blueReefLeftPipeline = 2;
        public static int blueReefRightPipeline = 3;
        public static int redReefCenterPipeline = 4;
        public static int redReefLeftPipeline = 5;
        public static int redReedRightPipeline = 6;

        /* Proportional limits for front limelight */
        public static double reefProportionalTx = 8;
        public static double algaeProportionalTx = 7.5;
    }
}