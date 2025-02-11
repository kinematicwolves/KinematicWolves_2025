// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private Funnel funnel;
  /** Creates a new IntakeCoral. */
  public IntakeCoral(Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnel.runMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
