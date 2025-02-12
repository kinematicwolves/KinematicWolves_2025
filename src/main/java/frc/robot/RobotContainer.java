// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    /* Swerve Speeds */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // 10% dead band
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);
    // private final CommandXboxController techController = new CommandXboxController(2);
    
    /* Choosers */
    // private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* Robot states */
    public boolean coralMode = true;
    public int scoringLevel = 0;

    /**Subsystems */
    private final Elevator elevatorSubsystem = new Elevator();

    /**Commands */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final SetElevatorPosition elevatorCoralLv4 = new SetElevatorPosition(elevatorSubsystem, 285.5);
    // private final SetElevatorPosition elevatorCoralLv3 = new SetElevatorPosition(elevatorSubsystem, 70);
    private final SetElevatorPosition elevatorCoralLv2 = new SetElevatorPosition(elevatorSubsystem, 142.5);
    private final SetElevatorPosition elevatorCoralLv1 = new SetElevatorPosition(elevatorSubsystem, 71.25);
    private final SetElevatorPosition elevatorHome     = new SetElevatorPosition(elevatorSubsystem, 0);
    



    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Test");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        /* Driver controls */
        // Note that X is defined as forward according to WPILib convention, and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake)); // A = X drivetrain wheels
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //  LB = Reset field-centric heading
        // drivetrain.registerTelemetry(logger::telemeterize);

        /* Operator controls */
        opController.back().onTrue(
            new InstantCommand(
                () -> {
                    coralMode = !coralMode;
                    SmartDashboard.putBoolean("Coral Mode Toggled:", coralMode); // Back = Coral/Algae mode toggle
                }
            )
        );

        opController.leftBumper().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 1) 
                        scoringLevel = 1;
                    System.out.println("Scoring level now set to: " + scoringLevel); // LB = Decrease scoring level
                }
            )
        );
        opController.rightBumper().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel += 1;
                    if (scoringLevel > 4)
                        scoringLevel = 4;
                    System.out.println("Scoring level now set to: " + scoringLevel); // RB = Increase scoring level
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

        opController.povDown().onTrue(elevatorCoralLv1);
        opController.povLeft().onTrue(elevatorCoralLv2);
        // opController.povUp().onTrue(elevatorCoralLv3);
        // opController.povRight().onTrue(elevatorCoralLv4);
        opController.a().onTrue(elevatorHome);

        /* Technician controls */
        // techController.povUp().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2));
        // techController.povDown().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // techController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // techController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // techController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // techController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return new InstantCommand();
    }
}
