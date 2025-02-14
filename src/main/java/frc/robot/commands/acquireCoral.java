// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AcquireCoral extends Command {
    /* Declare all the things this command needs */
    private Gripper gripperSubsystem;
    private double rollerSpeed;

    /**
     * Intake coral with tof sensor ending the command.
     * @param gripperSub subsystem, gripper subsystem
     * @param commandedRollerSpeed double, percentage output
     */
    public AcquireCoral(Gripper gripperSub, double commandedRollerSpeed) {
        // set our local variables
        gripperSubsystem = gripperSub;
        rollerSpeed = commandedRollerSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        gripperSubsystem.setRollerSpeed(rollerSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.setRollerSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return gripperSubsystem.hasCoral(); 
    }
}
