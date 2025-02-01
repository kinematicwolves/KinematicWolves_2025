// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AutoAlignTx extends Command {
  private Vision vision;
  private CommandSwerveDrivetrain drivetrain;
  private String limelight;
  private double negativeTolerance;
  private double positiveTolerance;
  private SwerveRequest.RobotCentric robotCentric;
  
  /** Creates a new AutoAlign. */
  public AutoAlignTx(CommandSwerveDrivetrain drivetrain, Vision vision, String limelight, double negativeTolerance, double positiveTolerance, SwerveRequest.RobotCentric robotCentric) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.robotCentric = robotCentric;
    this.limelight = limelight;
    this.negativeTolerance = negativeTolerance;
    this.positiveTolerance = positiveTolerance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    // addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // double driveSpeed = vision.getTx(limelight)/20;
    // if (vision.xOvershot(limelight, positiveTolerance)) {
    //   driveSpeed = -0.2;
    // }
    // else if (vision.xUnderShot(limelight, negativeTolerance)) {
    //   driveSpeed = 0.2;
    // }
    // else {
    //   driveSpeed = 0;
    // }
    drivetrain.applyRequest(() -> robotCentric.withRotationalRate(0.5));
    // System.out.println(driveSpeed);
    System.out.println("test");
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> robotCentric.withRotationalRate(0));
    System.out.println("Interupted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
