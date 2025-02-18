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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.IndexCoral;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RunRoller;
import frc.robot.commands.RunWrist;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SetWristPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;

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
    private final CommandXboxController techController = new CommandXboxController(2);
    
    /* Choosers */
    // private final SendableChooser<Command> autoChooser;
    
    /* Robot states */
    public boolean coralMode = true;
    public int scoringLevel = 1;

    /**Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevatorSubsystem = new Elevator();
    private final Gripper gripperSubsystem = new Gripper();
    private final Wrist wristSubsystem = new Wrist();

    /* Commands */
    // Swerve
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Elevator
    private final SetElevatorPosition elevatorCoralLv4    = new SetElevatorPosition(elevatorSubsystem, 289.5);
    private final SetElevatorPosition elevatorCoralLv3    = new SetElevatorPosition(elevatorSubsystem, 161.8);
    private final SetElevatorPosition elevatorCoralLv2    = new SetElevatorPosition(elevatorSubsystem, 50.5);
    private final SetElevatorPosition elevatorCoralLv1    = new SetElevatorPosition(elevatorSubsystem, 71.25);

    // private final SetElevatorPosition elevatorHome        = new SetElevatorPosition(elevatorSubsystem, 0);

    private final SetElevatorPosition elevatorShallowCage = new SetElevatorPosition(elevatorSubsystem, 142.5);

    private final SetElevatorPosition elevatorAlgaeLvl1   = new SetElevatorPosition(elevatorSubsystem, -81.5);
    private final SetElevatorPosition elevatorAlgaeLvl2   = new SetElevatorPosition(elevatorSubsystem, -18.5);
    private final SetElevatorPosition elevatorScoreAlgae  = new SetElevatorPosition(elevatorSubsystem, -91.5);

    // Wrist
    // private final SetWristPosition wristHome       = new SetWristPosition(wristSubsystem, -2);
    // private final SetWristPosition wristReady      = new SetWristPosition(wristSubsystem, 10);

    private final SetWristPosition wristCoralLvl1  = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl2  = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl3  = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl4  = new SetWristPosition(wristSubsystem, 10);

    private final SetWristPosition wristAlgaeLvl1  = new SetWristPosition(wristSubsystem, 49);
    private final SetWristPosition wristAlgaeLvl2  = new SetWristPosition(wristSubsystem, 30);
    private final SetWristPosition wristScoreAlgae = new SetWristPosition(wristSubsystem, 60);

    // Gripper
    private final AcquireCoral acquireCoral = new AcquireCoral(gripperSubsystem, 0.1);
    private final IndexCoral indexCoral     = new IndexCoral(gripperSubsystem, 0.1);
    // private final RunRoller intakeCoral      = new RunRoller(gripperSubsystem, 0.1);
    private final RunRoller outTakeCoral     = new RunRoller(gripperSubsystem, 0.1);

    private final RunRoller intakeAlgae      = new RunRoller(gripperSubsystem, -0.3);
    private final RunRoller outTakeAlgae     = new RunRoller(gripperSubsystem, 0.5);

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
        drivetrain.registerTelemetry(logger::telemeterize);

    /* Operator controls */
        opController.start().onTrue( //Coral Mode Toggle
            new InstantCommand(
                () -> {
                    coralMode = !coralMode;
                    SmartDashboard.putBoolean("Coral Mode Toggled:", coralMode);
                }
            )
        );
        opController.povDown().onTrue( //Decrease lvl
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 1) 
                        scoringLevel = 1;
                    System.out.println("Scoring level now set to: " + scoringLevel); // Down dpad = Decrease scoring level
                }
            )
        );
        opController.povUp().onTrue( //Increase lvl
            new InstantCommand(
                () -> {
                    scoringLevel += 1;
                    if (scoringLevel > 4)
                        scoringLevel = 4;
                    System.out.println("Scoring level now set to: " + scoringLevel); // Up dpad = Increase scoring level
                }
            )
        );

        Trigger coralModeTrigger = new Trigger(() -> coralMode);
        // Trigger scoringLevel0    = new Trigger(() -> scoringLevel == 0);
        Trigger scoringLevel1    = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2    = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3    = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4    = new Trigger(() -> scoringLevel == 4);


        // these compositions work fine
        // opController.y().onTrue(new SetWristPosition(wristSubsystem, 10).andThen(elevatorCoralLv1));
        // opController.y().onFalse(new SetElevatorPosition(elevatorSubsystem, 0).andThen(new SetWristPosition(wristSubsystem, -2)));

        // opController.x().onTrue(new SetWristPosition(wristSubsystem, 10).andThen(elevatorCoralLv2));
        // opController.x().onFalse(new SetElevatorPosition(elevatorSubsystem, 0).andThen(new SetWristPosition(wristSubsystem, -2)));

        // opController.a().onTrue(new SetWristPosition(wristSubsystem, 10).andThen(elevatorCoralLv3));
        // opController.a().onFalse(new SetElevatorPosition(elevatorSubsystem, 0).andThen(new SetWristPosition(wristSubsystem, -2)));

        // opController.b().onTrue(new SetWristPosition(wristSubsystem, 10).andThen(elevatorCoralLv4));
        // opController.b().onFalse(new SetElevatorPosition(elevatorSubsystem, 0).andThen(new SetWristPosition(wristSubsystem, -2)));

        opController.rightBumper().onTrue(acquireCoral.andThen(indexCoral).andThen(new SetWristPosition(wristSubsystem, 10)));

        // these compositions also work fine
        opController.a()
            .and(scoringLevel1)
            .onTrue(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(elevatorCoralLv1)
                .andThen(wristCoralLvl1)
            )
            .onFalse(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(new SetElevatorPosition(elevatorSubsystem, 0))
                .andThen(new SetWristPosition(wristSubsystem, -2))
            );

        opController.a()
            .and(scoringLevel2)
            .onTrue(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(elevatorCoralLv2)
                .andThen(wristCoralLvl2)
            )
            .onFalse(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(new SetElevatorPosition(elevatorSubsystem, 0))
                .andThen(new SetWristPosition(wristSubsystem, -2))
            );

        opController.a()
            .and(scoringLevel3)
            .onTrue(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(elevatorCoralLv3)
                .andThen(wristCoralLvl3)
            )
            .onFalse(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(new SetElevatorPosition(elevatorSubsystem, 0))
                .andThen(new SetWristPosition(wristSubsystem, -2))
                );

        opController.a()
            .and(scoringLevel4)
            .onTrue(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(elevatorCoralLv4)
                .andThen(wristCoralLvl4)
            )
            .onFalse(
                new SetWristPosition(wristSubsystem, 10)
                .andThen(new SetElevatorPosition(elevatorSubsystem, 0))
                .andThen(new SetWristPosition(wristSubsystem, -2))
            );

        opController.leftBumper().whileTrue(outTakeCoral);

        /* Technician controls */
        techController.start().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2));
        techController.back().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2));

        // techController.povDown().whileTrue(new RunWrist(wristSubsystem, -0.2));
        // techController.povUp().whileTrue(new RunWrist(wristSubsystem, 0.2));

        // techController.leftBumper().whileTrue(new RunRoller(gripperSubsystem, 0.2));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return new InstantCommand();
    }
}
