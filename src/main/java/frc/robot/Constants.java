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
        public static final int liftB_Id = 52;

        /* Motor Current Limit */
        public static final int currentLimit = 40;

        /* Elevator Height Limit */
        public static final double fwdSoftLimitNum = 300; // TODO: Config to max height

        /* Motor Idle Modes */
        public static final IdleMode defaultIdleMode = IdleMode.kBrake;

        /* PIDS */
        public static final double kP = 0.125;

        /* Encoder Error */
        public static final double encoderSetPointError = 25; // TODO: Config to precise positioning
    }
}
