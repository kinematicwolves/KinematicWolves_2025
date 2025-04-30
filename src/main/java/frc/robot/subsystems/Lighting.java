// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingProfile;

public class Lighting extends SubsystemBase {
  private CANdle candle = new CANdle(LightingProfile.canID, "canivore1");
  private Animation animation = null;

  /** Creates a new Lighting. */
  public Lighting() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = LightingProfile.brightnessScalar;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;

    candle.configAllSettings(cfg);
    candle.configLEDType(LEDStripType.GRB);

    setStillRainbowLightShow();
  }

  public void setGreenLightShow() {
    animation = new StrobeAnimation(0, 255, 0, 100, 0.1, LightingProfile.ledNum);
  }

  public void setRedLightShow() {
    animation = new TwinkleAnimation(225, 0, 0, 100, 1, LightingProfile.ledNum, TwinklePercent.Percent100);
  }

  public void setWhiteLarsonLightshow() {
    animation = new LarsonAnimation(255, 255, 255, 255, 0.8, LightingProfile.ledNum, BounceMode.Front, 25);
  }

  public void setLvl0LightShow() {
    animation = new RainbowAnimation(1, 0.9, LightingProfile.ledNum);
  }

  public void setLvl1LightShow() {
    animation = new TwinkleAnimation(255, 0, 255, 50, 1, 14, TwinklePercent.Percent100);
  }

  public void setLvl2Lightshow() {
    animation = new TwinkleAnimation(255, 0, 255, 50, 1, 24, TwinklePercent.Percent100);
  }

  public void setLvl3LightShow() {
    animation = new TwinkleAnimation(255, 0, 255, 50, 1, 34, TwinklePercent.Percent100);
  }

  public void setLvl4LightShow() {
    animation = new TwinkleAnimation(255, 0, 255, 50, 1, 44, TwinklePercent.Percent100);
  }

  public void setStillRainbowLightShow() {
    animation = new RainbowAnimation(0.1, 0.1, LightingProfile.ledNum);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (animation != null) {
      candle.animate(animation);
    }
  }
}
