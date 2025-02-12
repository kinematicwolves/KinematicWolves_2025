// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingProfile;

public class Lighting extends SubsystemBase {
  private CANdle chassisCandle = new CANdle(LightingProfile.chassisCandleId);
  private CANdle elevatorCandle = new CANdle(LightingProfile.elevatorCandleId);

  /** Creates a new Lighting. */
  public Lighting() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = LightingProfile.kBrightnessScalar;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;

    chassisCandle.configAllSettings(cfg);
    chassisCandle.configLEDType(LEDStripType.RGB);
    elevatorCandle.configAllSettings(cfg);
    elevatorCandle.configLEDType(LEDStripType.RGB);
  }

  /**
   * Sets commanded candle animation to red.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setRedLightShow(Animation commandedCandle, int ledCount) {
    //commandedCandle = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    commandedCandle = new ColorFlowAnimation(225, 0, 0, 100, 0.8, ledCount, Direction.Forward);
    // commandedCandle = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // commandedCandle = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // commandedCandle = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets commanded candle animation to blue.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setBlueLightShow(Animation commandedCandle, int ledCount) {
    //commandedCandle = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    commandedCandle = new ColorFlowAnimation(0, 0, 225, 100, 0.8, ledCount, Direction.Forward);
    // commandedCandle = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // commandedCandle = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // commandedCandle = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets commanded candle animation to green.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setGreenLightShow(Animation commandedCandle, int ledCount) {
    //commandedCandle = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    commandedCandle = new ColorFlowAnimation(0, 225, 0, 100, 0.8, ledCount, Direction.Forward);
    // commandedCandle = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // commandedCandle = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // commandedCandle = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets commanded candle animation to fire.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setFireLightShow(Animation commandedCandle, double brightness, int ledCount) {
    commandedCandle = new FireAnimation(brightness, 0.5, ledCount, 0.7, 0.5);
  }

  /**
   * Sets commanded candle animation to rainbow.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setRainbowLightShow(Animation commandedCandle, double brightness, int ledCount) {
    commandedCandle = new RainbowAnimation(brightness, 0.3, ledCount);
  }

  /**
   * Sets commanded candle animation to rgb fade.
   * @param commandedCandle Animation, commanded candle
   * @param ledCount int, number of leds (including candle leds)
   */
  public void setRgbFadeLightShow(Animation commandedCandle, double brightness, int ledCount) {
    commandedCandle = new RgbFadeAnimation(brightness, 0.3, ledCount);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    if (LightingProfile.chassisAnimation != null){
      elevatorCandle.animate(LightingProfile.chassisAnimation);
    }
    if (LightingProfile.elevatorAnimation != null){
      chassisCandle.animate(LightingProfile.elevatorAnimation);
    }
  }
}