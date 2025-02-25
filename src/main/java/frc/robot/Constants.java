// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Constants {
    public class ElevatorProfile {
        /* CAN Id's */
        public static final int liftA_Id = 50;
        public static final int liftB_Id = 51;

        /* Motor Current Limit */
        public static final int currentLimit = 35;

        /* Elevator Height Limit */
        public static final double fwdSoftLimitNum = 300; // TODO: Config to max height

        /* Motor Idle Modes */
        public static final IdleMode defaultIdleMode = IdleMode.kBrake;

        /* PIDS */
        public static final double kP = 2;

        /* Encoder Error */
        public static final double encoderSetPointError = 1; // TODO: Config to precise positioning
    }

    public class GripperProfile {
        /* CAN Id */
        public static final int motorId = 56;
        public static final int tofId = 57;

        /* Motor Current Limit */
        public static final double currentLimit = 10;

        /* Sensor Range Values */
        public static final double tofCoralRange = 75;
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