// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristPosition extends Command {
  private Wrist wrist;
  private double setPoint;
  private int slot;

  /**
  * Moves the wrist to the set point. Finished when at commanded set point.
  * @param wrist subsystem, wrist subsystem
  * @param setPoint double, falcon motor encoder revolutions
  * @param slot pid slot config to use
  */
  public SetWristPosition(Wrist wrist, double setPoint, int slot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.setPoint = setPoint;
    this.slot = slot;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setWristPos(setPoint, slot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atPosition();
  }
}
