// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionProfile;
import frc.robot.generated.LimelightHelpers;

public class Vision extends SubsystemBase {
  
  /* Creates a new Vision. */
  public Vision() {
    DriverStation.getAlliance().get();
    setLimelightPipeline(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test);
    setLimelightPipeline(VisionProfile.rearLimelight, VisionProfile.stationTestPipeline_Test);
  }

  /**
   * Sets commanded pipeline setup to limelight.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   */
  private void setLimelightPipeline(String limelight, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelight, pipeline);
  }
  
  /**
   * Retrieves limelight Tx coordinate value based off alliance color.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return double, Tx
   */
  public double getTx(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -LimelightHelpers.getTX(limelight);
    }
    else {
      return LimelightHelpers.getTX(limelight);
    }
  }

  /**
   * Retrieves limelight Ty coordinate value based off alliance color.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return double, Ty
   */
  public double getTy(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -LimelightHelpers.getTY(limelight);
    }
    else {
      return LimelightHelpers.getTY(limelight);
    }
  }

  /**
   * Retrieves limelight Ta coordinate value based off alliance color.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return double, Ta
   */
  public double getTa(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -LimelightHelpers.getTA(limelight);
    }
    else {
      return LimelightHelpers.getTA(limelight);
    }
  }

  /**
   * Checks if limelights target Tx is within -1 and 1.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return true if Tx is between -1 & 1, else false
   */
  public boolean isTxAligned(String limelight, int pipeline) {
    double Tx = getTx(limelight, pipeline);

    if ((Tx >= -1) && (Tx <= 1)) {
      return true;
    }
    else {
      return false;
    }
  }

    /**
   * Checks if limelights target Ty is within -1 and 1.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return true if Ty is between -1 & 1, else false
   */
  public boolean isTyAligned(String limelight, int pipeline) {
    double Ty = getTy(limelight, pipeline);

    if ((Ty >= -1) && (Ty <= 1)) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks if limelights target Ta is within -1 and 1.
   * @param limelight String, limelight name
   * @param pipeline int, commanded pipeline
   * @return true if Ta is between -1 & 1, else false
   */
  public boolean isTaAligned(String limelight, int pipeline) {
    double Ta = getTa(limelight, pipeline);

    if ((Ta >= -1) && (Ta <= 1)) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Front limelight coordinate feedback
    SmartDashboard.putNumber("Reef Tx", getTx(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));
    SmartDashboard.putNumber("Reef Ty", getTy(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));
    SmartDashboard.putNumber("Reef Ta", getTa(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));

    // Rear limelight coordinate feedback
    SmartDashboard.putNumber("Station Tx", getTx(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putNumber("Station Ty", getTy(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putNumber("Station Ta", getTa(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));

    // Front limelight alignment feedback
    SmartDashboard.putBoolean("Front limelight Tx Aligned", isTxAligned(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));
    SmartDashboard.putBoolean("Front limelight Ty Aligned", isTyAligned(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));
    SmartDashboard.putBoolean("Front limelight Ta Aligned", isTaAligned(
      VisionProfile.frontLimelight, 
      VisionProfile.reefPipeline_Test));

    // Rear limelight alignment feedback
    SmartDashboard.putBoolean("Rear limelight Tx Aligned", isTxAligned(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putBoolean("Rear limelight Ty Aligned", isTyAligned(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putBoolean("Rear limelight Ta Aligned", isTaAligned(
      VisionProfile.rearLimelight, 
      VisionProfile.stationTestPipeline_Test));
  }
}
