// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Constants {
    public class DriverProfile {
        /* Angular speed multiplier */
        public static final double kRotationMagnitude = 0.5;

        /* Driver station ports */
        public static final int driverPortNum = 0;
        public static final int operatorPortNum = 1;
        public static final int technicianPortNum = 2;
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
        public static final double kP = 0.7; // TODO: config for new gear ratio
        public static final double kI = 0; // TODO: config for new gear ratio
        public static final double kD = 0.1; // TODO: config for new gear ratio

        /* Encoder Error */
        public static final double encoderSetPointError = 1; // TODO: Config to precise positioning

        /* Positions */
        public static final double coralLvl1Pos = 10; // TODO: config for new gear ratio
        public static final double coralLvl2Pos = 50.5; // TODO: config for new gear ratio
        public static final double coralLvl3Pos = 156; // TODO: config for new gear ratio
        public static final double coralLvl4Pos = 295; // TODO: config for new gear ratio

        public static final double algaeLvl1Pos = 113; // TODO: config for new gear ratio
        public static final double algaeLvl2Pos = 199; // TODO: config for new gear ratio
        public static final double algaeScorePos = 35; // TODO: config for new gear ratio

        public static final double shallowCagePos = 128.4; // TODO: config for new gear ratio
    }

    public class WristProfile {
        /* CAN Id */
        public static final int motorID = 60;

        /* Motor Current Limit */
        public static final int currentLimit = 20;

        /* PIDs */
        public static final double kP = 0.035;
        public static final double kI = 0;
        public static final double kD = 6;

        /* Encoder Error */
        public static final double encoderSetPointError = 0.5;

        /* Positions */ //TODO: Might need to config for new gear ratio
        public static final double coralLvl1Pos = 4;
        public static final double coralLvl2Pos = 4;
        public static final double coralLvl3Pos = 7;
        public static final double coralLvl4Pos = 10;

        public static final double algaeLvl1Pos = 47;
        public static final double algaeLvl2Pos = 47;
        public static final double algaeScorePos = 54;

        public static final double shallowCagePos = 0;
    }

    public class GripperProfile {
        /* CAN Id */
        public static final int motorId = 56;
        public static final int tofId = 57;

        /* Motor Current Limit */
        public static final double currentLimit = 15;

        /* Sensor Range Values */
        public static final double tofCoralRange = 75;

        /* Default Speeds (Percentage Outputs) */
        public static final double acquireCoralSpeed = 0.11;
        public static final double indexCoralSpeed = 0.09;

        public static final double intakeAlgaeSpeed = -0.4;
        public static final double holdAlgaeOutput = -0.18;
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

        public static int stationTestPipeline_Test = 0;

        /* Proportional limits for front limelight */
        public static int frontProportionalTx = 7;
        public static int frontProportionalTy = 20;
        public static int frontProportionalTa = 20;

        /* Proportional limits for rear limelight */
        public static int rearProportionalTx = 20;
        public static int rearProportionalTy = 20;
        public static int rearProportionalTa = 20;
    }
}