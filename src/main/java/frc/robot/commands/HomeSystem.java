// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeSystem extends SequentialCommandGroup {
    /** Creates a new HomeSystem. */
    public HomeSystem(Wrist wristSubsystem, Elevator elevatorSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetWristPosition(wristSubsystem, 10), // TODO: Put Number into Constants.WristProfile
            new SetElevatorPosition(elevatorSubsystem, 0), // TODO: Put Number into Constants.ElevatorProfile
            new SetWristPosition(wristSubsystem, -2) // TODO: Put Number into Constants.WirstProfile
        );
    }
}
