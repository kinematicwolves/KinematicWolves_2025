// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotStates.SetDisabledState;
import frc.robot.RobotStates.SetTeleOpState;
import frc.robot.RobotStates.SetTestState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lighting;

public class RobotContainer {
    /* Swerve */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);
    private final CommandXboxController techController = new CommandXboxController(2);
    
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    private final Lighting lightingSubsystem = new Lighting();
    
    /* Robot states */
    private boolean coralMode = true;

    /* Commands */

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        /* Driver controls */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* Other */
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    // Command to reset robot to initial teleop light show/state
    public Command getTeleOpInitCommand() {
        return new SetTeleOpState(lightingSubsystem);     
    }

    // Command to reset robot to initial testOp light show/state
    public Command getTestOpInitCommand() {
        return new SetTestState(lightingSubsystem);
    }

    // Command to reset robot to initial disable light show/state
    public Command getDisabledCommandInitCommand() {
        return new SetDisabledState(lightingSubsystem);
    }
}