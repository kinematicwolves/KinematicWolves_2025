// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveProfile;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignTx extends Command {
  private Swerve swerve;
  private Vision vision;
  private String limelight;
  private double negativeTolerance;
  private double positiveTolerance;
  
  /** Creates a new AutoAlign. */
  public AutoAlignTx(Swerve swerve, Vision vision, String limelight, double negativeTolerance, double positiveTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.vision = vision;
    this.limelight = limelight;
    this.negativeTolerance = negativeTolerance;
    this.positiveTolerance = positiveTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.swerve.setAngularControllerOnOff(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.xOvershot(limelight, positiveTolerance)) {
      swerve.setVelocityX(SwerveProfile.defaultSwerveVelocity);
    }
    else if (vision.xUnderShot(limelight, negativeTolerance)) {
      swerve.setVelocityX(SwerveProfile.defaultSwerveVelocity);
    }
    else {
      swerve.setVelocityX(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (vision.xAtSetPoint(limelight, negativeTolerance, positiveTolerance) == true) {
      swerve.setAngularControllerOnOff(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
