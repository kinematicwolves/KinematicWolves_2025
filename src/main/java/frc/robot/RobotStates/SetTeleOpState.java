// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotStates;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightingProfile;
import frc.robot.subsystems.Lighting;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTeleOpState extends Command {
  private Lighting lighting;
  private int matchTimer;

  /** Creates a new TeleOpLightShow. */
  public SetTeleOpState(Lighting lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lighting = lighting;
    matchTimer = 135000;
    addRequirements(lighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    matchTimer = 135000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    matchTimer -= 20;
    SmartDashboard.putNumber("Match Time Remaining", Units.millisecondsToSeconds(matchTimer));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SetDisabledState(lighting);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
