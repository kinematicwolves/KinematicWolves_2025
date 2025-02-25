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
  private CANdle elevatorCandle = new CANdle(LightingProfile.elevatorCandleId);
  private Animation elevatorAnimation = null;

  /** Creates a new Lighting. */
  public Lighting() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = LightingProfile.kBrightnessScalar;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;

    elevatorCandle.configAllSettings(cfg);
    elevatorCandle.configLEDType(LEDStripType.RGB);
  }

  /**
   * Sets elevator candle animation to red.
   */
  public void setRedLightShow() {
    // elevatorAnimation = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(225, 0, 0, 100, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // elevatorAnimation = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets elevator candle animation to blue.
   */
  public void setBlueLightShow() {
    // elevatorAnimation = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(0, 0, 225, 100, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // elevatorAnimation = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets elevator candle animation to green.
   */
  public void setGreenLightShow() {
    //elevatorAnimation = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(0, 225, 0, 100, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // elevatorAnimation = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets elevator candle animation to teal.
   */
  public void setTealLightShow() {
    //elevatorAnimation = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(0, 128, 128, 100, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // elevatorAnimation = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets elevator candle animation to white.
   */
  public void setWhiteLightShow() {
    //elevatorAnimation = new TwinkleAnimation(225, 0, 0, 100, 1, ledCount, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(225, 225, 225, 225, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 0, 100, 1, ledCount, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 0, 100, 0.5, ledCount);
    // elevatorAnimation = new StrobeAnimation(225, 0, 0, 100, 0.5, ledCount);
  }

  /**
   * Sets elevator candle animation to purple.
   */
  public void setPurpleLightShow() {
    // elevatorAnimation = new TwinkleAnimation(225, 0, 225, 100, 100, 1, LightingProfile.elevatorNumLedStrip, TwinklePercent.Percent100);
    elevatorAnimation = new ColorFlowAnimation(225, 0, 225, 100, 0.8, LightingProfile.elevatorNumLedStrip, Direction.Forward);
    // elevatorAnimation = new LarsonAnimation(225, 0, 225, 100, 100, 1, LightingProfile.elevatorNumLedStrip, BounceMode.Front, 5);
    // elevatorAnimation = new SingleFadeAnimation(225, 0, 225, 100, 100, 0.5, LightingProfile.elevatorNumLedStrip);
    // elevatorAnimation = new StrobeAnimation(225, 0, 225, 100, 100, 0.5, LightingProfile.elevatorNumLedStrip);
  }

  /**
   * Sets elevator candle animation to fire.
   * @param brightness double, led brightness percentage
   * @param speed double, how fast the rainbow travels through the led [0,1]
   */
  public void setFireLightShow(double brightness, double speed) {
    elevatorAnimation = new FireAnimation(brightness, speed, LightingProfile.elevatorNumLedStrip, 0.7, 0.5);
  }

  /**
   * Sets elevator candle animation to rainbow.
   * @param brightness double, led brightness percentage
   * @param speed double, how fast the rainbow travels through the led [0,1]
   */
  public void setRainbowLightShow(double brightness, double speed) {
    elevatorAnimation = new RainbowAnimation(brightness, speed, LightingProfile.elevatorNumLedStrip);
  }

  /**
   * Sets elevator candle animation to rgb fade.
   * @param brightness double, led brightness percentage
   * @param speed double, how fast the rainbow travels through the led [0,1]
   */
  public void setRgbFadeLightShow(double brightness, double speed) {
    elevatorAnimation = new RgbFadeAnimation(brightness, speed, LightingProfile.elevatorNumLedStrip);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    if (elevatorAnimation != null){
      elevatorCandle.animate(elevatorAnimation);
    }
  }
}