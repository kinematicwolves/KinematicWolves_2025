// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lighting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetDisableLightShow extends Command {
  private Lighting lighting;

  /** Creates a new SetDisableLightShow. */
  public SetDisableLightShow(Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lighting = lighting;
    addRequirements(lighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lighting.setStillRainbowLightShow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
