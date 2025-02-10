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
   * Sets pipeline for front limelight.
   * 
   * @param pipeline int
   */
  private void setLimelightPipeline(String limelight, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelight, pipeline);
  }
  
  public double getTx(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return -LimelightHelpers.getTX(limelight);
    }
    else {
      return LimelightHelpers.getTX(limelight);
    }
  }


  public double getTy(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getTY(limelight);
    }
    else {
      return -LimelightHelpers.getTY(limelight);
    }
  }


  public double getTa(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getTA(limelight);
    }
    else {
      return -LimelightHelpers.getTA(limelight);
    }
  }

  public boolean isTxAligned(String limelight, int pipeline) {
    double Tx = getTx(limelight, pipeline);

    return (Tx <= -1) && (Tx >= 1);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    SmartDashboard.putNumber("Reef Tx", getTx(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test));
    SmartDashboard.putNumber("Reef Ty", getTy(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test));
    SmartDashboard.putNumber("Reef Ta", getTa(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test));

    SmartDashboard.putNumber("Station Tx", getTx(VisionProfile.rearLimelight, VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putNumber("Station Ty", getTy(VisionProfile.rearLimelight, VisionProfile.stationTestPipeline_Test));
    SmartDashboard.putNumber("Station Ta", getTa(VisionProfile.rearLimelight, VisionProfile.stationTestPipeline_Test));

    SmartDashboard.putBoolean("Tx Aligned", isTxAligned(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test));

  }
}
