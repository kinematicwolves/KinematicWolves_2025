// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorProfile;
import frc.robot.Constants.WristProfile;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeSystemAlgae extends SequentialCommandGroup {
    /** Creates a new HomeSystem. */
    public HomeSystemAlgae(Wrist wristSubsystem, Elevator elevatorSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetWristPosition(wristSubsystem, WristProfile.algaeHomePos, 0),
            new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.homePos)
        );
    }
}
