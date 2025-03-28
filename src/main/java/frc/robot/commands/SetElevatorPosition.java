// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
    /* Declare all the things this command needs */
    private Elevator elevatorSubsystem;
    private double setPoint;

    /**
     * Runs elevator to commanded set point. Finished when at commanded set point.
     * @param elevatorSub subsystem, elevator subsystem
     * @param commandedSetPoint double, neo motor encoder revolutions
     */
    public SetElevatorPosition(Elevator elevatorSub, double commandedSetPoint) {
      elevatorSubsystem = elevatorSub;
      setPoint = commandedSetPoint;
      
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      elevatorSubsystem.setPosition(setPoint);
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
        return elevatorSubsystem.atPosition();
    }
}
