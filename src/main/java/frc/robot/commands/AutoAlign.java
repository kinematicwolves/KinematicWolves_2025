// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AutoAlign extends Command {
  private Vision vision;
  private CommandSwerveDrivetrain drivetrain;
  private String limelight;
  private double negativeTolerance;
  private double positiveTolerance;
  private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity); 
  
  /**
   * Auto Aligns robot to commanded target.
   * 
   * @param drivetrain Subsystem, CommandSwerve Drivetrain subsystem
   * @param vision Subsystem, Vision Subsystem
   * @param limelight String, limelight name (VisionProfile constants)
   * @param negativeTolerance double, target negative tolerance (VisionProfile constants)
   * @param positiveTolerance double, target positive tolerance (VisionProfile constants)
   */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, Vision vision, String limelight, double negativeTolerance, double positiveTolerance) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.negativeTolerance = negativeTolerance;
    this.positiveTolerance = positiveTolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (vision.xOvershot(limelight, positiveTolerance)) {
      drivetrain.applyRequest(() -> robotCentric.withVelocityX(-0.3));
    }
    else if (vision.xUnderShot(limelight, negativeTolerance)) {
      drivetrain.applyRequest(() -> robotCentric.withVelocityX(0.3));
    }
    else {
      drivetrain.applyRequest(() -> robotCentric.withVelocityX(0));
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> robotCentric.withVelocityX(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
