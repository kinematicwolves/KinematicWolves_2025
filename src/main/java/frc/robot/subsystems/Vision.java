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
    getAllianceColor();
    setLimelightPipeline(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test);
    setLimelightPipeline(VisionProfile.rearLimelight, VisionProfile.stationTestPipeline_Test);
  }

  /**
   * Gets driver station assigned color.
   * 
   * @return DriverStation.Alliance, blue or red.
   */
  private DriverStation.Alliance getAllianceColor() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return DriverStation.Alliance.Blue;
    }
    else {
      return DriverStation.Alliance.Red;
    }
  }

  /**
   * Sets pipeline for front limelight.
   * 
   * @param pipeline int
   */
  private void setLimelightPipeline(String limelight, int pipeline) {
    LimelightHelpers.setPipelineIndex(limelight, pipeline);
  }
  
  /**
  * Returns the 2d translation from the limelight position estimation object.
  *
  * @param limelight String, limelight name
  * @param pipeline int, limelight pipeline
  * @return Translation2d
  */
  public double getTranslationX(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (getAllianceColor() == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight).pose.getTranslation().getX();
    }
    else {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(limelight).pose.getTranslation().getX();
    }
  }

  /**
  * Returns the 2d translation from the limelight position estimation object.
  *
  * @param limelight String, limelight name
  * @param pipeline int, limelight pipeline
  * @return Translation2d
  */
  public double getTranslationY(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (getAllianceColor() == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight).pose.getTranslation().getY();
    }
    else {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(limelight).pose.getTranslation().getY();
    }
  }

  /**
  * Returns the 2d translation from the limelight position estimation object.
  *
  * @param limelight String, limelight name
  * @param pipeline int, limelight pipeline
  * @return Translation2d
  */
  public double getRotation2d(String limelight, int pipeline) {
    setLimelightPipeline(limelight, pipeline);
    if (getAllianceColor() == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight).pose.getRotation().getRadians()/1.2;
    }
    else {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(limelight).pose.getRotation().getRadians()/1.2;
    }
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    SmartDashboard.putNumber("Front Limelight Rad", getTranslationY(VisionProfile.frontLimelight, VisionProfile.reefPipeline_Test));
  }
}
