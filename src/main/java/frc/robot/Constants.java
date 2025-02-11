// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class VisionProfile {
        /* Limelight names */
        public static String frontLimelight = "limelight-front";
        public static String rearLimelight = "limelight-rear";

        /* Calibrated limelight pipelines */
        public static int reefPipeline_Test = 0;
        public static int stationTestPipeline_Test = 0;

        /* Proportional limits for front limelight */
        public static int frontProportionalTx = 20;
        public static int frontProportionalTy = 20;
        public static int frontProportionalTa = 20;

        /* Proportional limits for rear limelight */
        public static int rearProportionalTx = 20;
        public static int rearProportionalTy = 20;
        public static int rearProportionalTa = 20;
    }
}
