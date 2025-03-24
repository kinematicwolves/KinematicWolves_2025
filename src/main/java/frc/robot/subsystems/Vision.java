// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionProfile;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  
  /* Creates a new Vision. */
  public Vision() {
    setLimelightPipeline(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test);

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
   * Retrieves reef Tx from left reef pipelines.
   * 
   * @param limelight String, front limelight
   * @return double, limelight Tx
   */
  public double getLeftReefTx(String limelight) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      setLimelightPipeline(limelight, VisionProfile.blueReefLeftPipeline);

      return -LimelightHelpers.getTX(limelight);
    }
    else {
      setLimelightPipeline(limelight, VisionProfile.redReefLeftPipeline);
      return -LimelightHelpers.getTX(limelight);
    }
  }

  /**
   * Retrieves reef Tx from right reef pipelines.
   * 
   * @param limelight String, front limelight
   * @return double, limelight Tx
   */
  public double getRightReefTx(String limelight) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      setLimelightPipeline(limelight, VisionProfile.blueReefRightPipeline);

      return -LimelightHelpers.getTX(limelight);
    }
    else {
      setLimelightPipeline(limelight, VisionProfile.redReedRightPipeline);
      return -LimelightHelpers.getTX(limelight);
    }
  }

  /**
   * Retrieves reef Tx from center reef pipelines.
   * 
   * @param limelight String, front limelight
   * @return double, limelight Tx
   */
  public double getCenterReefTx(String limelight) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      setLimelightPipeline(limelight, VisionProfile.blueReefCenterPipeline);

      return -LimelightHelpers.getTX(limelight);
    }
    else {
      setLimelightPipeline(limelight, VisionProfile.redReefCenterPipeline);
      return -LimelightHelpers.getTX(limelight);
    }
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {}
}
