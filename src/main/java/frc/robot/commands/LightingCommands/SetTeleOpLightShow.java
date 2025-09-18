// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LightingProfile;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTeleOpLightShow extends Command {
  private Lighting lighting;
  private Elevator elevator;
  private Wrist wrist;
  private Gripper gripper;

  /** Creates a new SetTeleOpLightShow. */
  public SetTeleOpLightShow(Lighting lighting, Elevator elevator, Wrist wrist, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lighting = lighting;
    this.elevator = elevator;
    this.wrist = wrist;
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((elevator.getPosition() <= 1) && (wrist.getWristPos() <= 1) && (gripper.getRollerCurrent() >= 1)) {
      lighting.setWhiteLarsonLightshow();
    }
    else {
      lighting.setLvl0LightShow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lighting.setStillRainbowLightShow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
