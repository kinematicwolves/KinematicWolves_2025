// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimberSpeed extends Command {
  private Climber climber;
  private double speed;
  /** Creates a new SetClimberSpeed. */
  public SetClimberSpeed(Climber climbersub,double commandSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climbersub;
    speed = commandSpeed;

    addRequirements(climbersub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
