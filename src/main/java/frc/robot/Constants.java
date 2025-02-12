// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.Animation;

/** Add your docs here. */
public class Constants {
    public class LightingProfile {
    /* Id's */
    public static final int chassisCandleId = 61;
    public static final int elevatorCandleId = 62;
    
    /* Brightness */
    public static final double kBrightnessScalar = 1;

    /* Total Number of Led's */
    public static final int chassisNumLedStrip = 88;
    public static final int elevatorNumLedStrip = 88;

    public static  Animation elevatorAnimation = null;
    public static Animation chassisAnimation = null;
    }
}
