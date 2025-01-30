// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionProfile;
import frc.robot.generated.LimelightHelpers;

public class Vision extends SubsystemBase {
  // objects to facilitate position estimates from limelight 
  private Debouncer alignmentFilter = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  /* Creates a new Vision. */
  public Vision() {
    setFrontLimelightPipeline(VisionProfile.reefPipeline_Test);
    setRearLimelightPipeline(VisionProfile.stationTestPipeline_Test);
  }

  /**
   * Sets pipeline for front limelight.
   * 
   * @param pipeline int
   */
  private void setFrontLimelightPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(VisionProfile.frontLimelight, pipeline);
  }

  /**
   * Sets pipeline for rear limelight.
   * 
   * @param pipeline int
   */
  private void setRearLimelightPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(VisionProfile.rearLimelight, pipeline);
  }

  /**
   * Retrieves Tx coordinate from limelight.
   * 
   * @param limelight String, limelight
   * @return double, limelight Tx unit value
   */
  private double getTx(String limelight) {
    return LimelightHelpers.getTX(limelight);
  }

  /**
   * Checks if limelight Tx coordinate is above set point positive tolerance.
   * 
   * @param limelight String, limelight
   * @param positiveTolerance double, positive tolerance set point 
   * @return true if limelight Tx is over set point tolerance, else false
   */
  public boolean xOvershot(String limelight, double positiveTolerance) {
    if (getTx(limelight) > positiveTolerance) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if  limelight Tx coordinate is below set point negative tolerance.
   * 
   * @param limelight String, limelight
   * @param negativeTolerance double, negative tolerance set point 
   * @return true if limelight Tx is under set point tolerance, else false
   */
  public boolean xUnderShot(String limelight, double negativeTolerance) {
    if (getTx(limelight) < negativeTolerance) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if limelight Tx coordinate is between negative tolerance set point and positive set point tolerance. 
   * 
   * @param limelight String, limelight
   * @param negativeTolerance double, negative tolerance set point
   * @param positiveTolerance double, positive tolerance set point
   * @return true if limelight Tx is between tolerance set points, else false
   */
  public boolean xAtSetPoint(String limelight, double negativeTolerance, double positiveTolerance) {
    if ((getTx(limelight) >= negativeTolerance) && (getTx(limelight) <= positiveTolerance)){
      return alignmentFilter.calculate(true);
    }
    else {
      return false;
    }
  }

  /**
   * Retrieves Tx coordinate from limelight.
   * 
   * @param limelight String, limelight
   * @return double, limelight Tx unit value
   */
  private double getTa(String limelight) {
    return LimelightHelpers.getTA(limelight);
  }

  /**
   * Checks if limelight Ta coordinate is above set point positive tolerance.
   * 
   * @param limelight String, limelight
   * @param positiveTolerance double, positive tolerance set point 
   * @return true if limelight Ta is over set point tolerance, else false
   */
  public boolean aOvershot(String limelight, double positiveTolerance) {
    if (getTa(limelight) > positiveTolerance) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if  limelight Ta coordinate is below set point negative tolerance.
   * 
   * @param limelight String, limelight
   * @param negativeTolerance double, negative tolerance set point 
   * @return true if limelight Ta is under set point tolerance, else false
   */
  public boolean aUnderShot(String limelight, double negativeTolerance) {
    if (getTa(limelight) < negativeTolerance) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if limelight Ta coordinate is between negative tolerance set point and positive set point tolerance. 
   * 
   * @param limelight String, limelight
   * @param negativeTolerance double, negative tolerance set point
   * @param positiveTolerance double, positive tolerance set point
   * @return true if limelight Ta is between tolerance set points, else false
   */
  public boolean aAtSetPoint(String limelight, double negativeTolerance, double positiveTolerance) {
    if ((getTa(limelight) >= negativeTolerance) && (getTx(limelight) <= positiveTolerance)){
      return alignmentFilter.calculate(true);
    }
    else {
      return false;
    }
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    /* Limelight measurements */
    SmartDashboard.putNumber("Front Limelight Tx", getTx(
      VisionProfile.frontLimelight));
    SmartDashboard.putNumber("Rear Limelight Tx", getTx(
      VisionProfile.rearLimelight));

    SmartDashboard.putNumber("Front Limelight Ta", getTa(
      VisionProfile.frontLimelight));
    SmartDashboard.putNumber("Back Limelight Ta", getTa(
      VisionProfile.rearLimelight));

    /* Alignment feedback */
    SmartDashboard.putBoolean("Robot aligned with right reef", xAtSetPoint(
      VisionProfile.frontLimelight, 
      VisionProfile.rightReefSetPointNegativeTolerance_Tx, 
      VisionProfile.rightReefSetPointPositiveTolerance_Tx));
    SmartDashboard.putBoolean("Robot aligned with left reef", xAtSetPoint(
      VisionProfile.frontLimelight, 
      VisionProfile.leftReefSetPointNegativeTolerance_Tx, 
      VisionProfile.leftReefSetPointPositiveTolerance_Tx));
    SmartDashboard.putBoolean("Robot Tx aligned with center reef", xAtSetPoint(
      VisionProfile.frontLimelight, 
      VisionProfile.centerReefSetPointNegativeTolerance_Tx, 
      VisionProfile.centerReefSetPointPositiveTolerance_Tx));
    SmartDashboard.putBoolean("Robot Ta aligned with center reed", aAtSetPoint(
      VisionProfile.frontLimelight, 
      VisionProfile.centerReefSetPointNegativeTolerance_Ta, 
      VisionProfile.centerReefSetPointPositiveTolerance_Ta));
    SmartDashboard.putBoolean("Robot aligned with station", xAtSetPoint(
      VisionProfile.rearLimelight, 
      VisionProfile.stationSetPointNegativeTolerance_Tx, 
      VisionProfile.stationSetPointPositiveTolerance_Tx));
  }
}
