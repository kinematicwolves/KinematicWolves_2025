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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
    
    /* Robot states */
    public boolean coralMode = true;
    public int scoringLevel = 0;

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

        /* Operator controls */
        opController.back().onTrue(
            new InstantCommand(
                () -> {
                    coralMode = !coralMode;
                    System.out.println("Coral Mode Toggled: " + coralMode);
                }
            )
        );

        opController.leftBumper().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 1) 
                        scoringLevel = 1;
                    System.out.println("Scoring level now set to: " + scoringLevel);
                }
            )
        );

        opController.rightBumper().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel += 1;
                    if (scoringLevel > 4)
                        scoringLevel = 4;
                    System.out.println("Scoring level now set to: " + scoringLevel);
                }
            )
        );

        Trigger coralModeTrigger = new Trigger(() -> coralMode);
        Trigger scoringLevel1    = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2    = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3    = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4    = new Trigger(() -> scoringLevel == 4);

        // examples of how to use these triggers
        opController.a().onTrue(
            new InstantCommand(() -> System.out.println("A Pressed - Default Action"))
        );

        coralModeTrigger.and(opController.a()).onTrue(
            new InstantCommand(() -> System.out.println("A Pressed - Coral Mode Action"))
        );

        opController.b().onTrue(
            new InstantCommand(() -> System.out.println("B Pressed - Default Action"))
        );

        coralModeTrigger.and(opController.b()).onTrue(
            new InstantCommand(() -> System.out.println("B Pressed - Coral Mode Action"))
        );

        /* Technician controls */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // techController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // techController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // techController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // techController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* Other */
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
