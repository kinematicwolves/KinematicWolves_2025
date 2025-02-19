// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.HomeSystem;
import frc.robot.commands.IndexCoral;
import frc.robot.commands.MoveToLevel;
import frc.robot.commands.SetRollerSpeed;
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
    private final SetElevatorPosition elevatorShallowCage = new SetElevatorPosition(elevatorSubsystem, 142.5);

    private final SetElevatorPosition elevatorAlgaeLvl1  = new SetElevatorPosition(elevatorSubsystem, -81.5);
    private final SetElevatorPosition elevatorAlgaeLvl2  = new SetElevatorPosition(elevatorSubsystem, -18.5);
    private final SetElevatorPosition elevatorScoreAlgae = new SetElevatorPosition(elevatorSubsystem, -91.5);

    // Wrist
    private final SetWristPosition wristAlgaeLvl1  = new SetWristPosition(wristSubsystem, 49);
    private final SetWristPosition wristAlgaeLvl2  = new SetWristPosition(wristSubsystem, 30);
    private final SetWristPosition wristScoreAlgae = new SetWristPosition(wristSubsystem, 60);

    // Wist and Elevator
    private final MoveToLevel moveCoraLevel1 = new MoveToLevel(wristSubsystem, elevatorSubsystem, 71.25,  4); // TODO: Put Number into Constants.ElevatorProfile
    private final MoveToLevel moveCoraLevel2 = new MoveToLevel(wristSubsystem, elevatorSubsystem, 50.5,   4); // TODO: Put Number into Constants.ElevatorProfile
    private final MoveToLevel moveCoraLevel3 = new MoveToLevel(wristSubsystem, elevatorSubsystem, 161.8,  4); // TODO: Put Number into Constants.ElevatorProfile
    private final MoveToLevel moveCoraLevel4 = new MoveToLevel(wristSubsystem, elevatorSubsystem, 289.5, 10); // TODO: Put Number into Constants.ElevatorProfile

    // Gripper
    private final AcquireCoral   acquireCoral = new AcquireCoral(gripperSubsystem, 0.1);
    private final IndexCoral     indexCoral   = new IndexCoral(gripperSubsystem, 0.1);
    private final SetRollerSpeed outTakeCoral = new SetRollerSpeed(gripperSubsystem, 0.1);

    private final SetRollerSpeed intakeAlgae  = new SetRollerSpeed(gripperSubsystem, -0.3);
    private final SetRollerSpeed outTakeAlgae = new SetRollerSpeed(gripperSubsystem, 0.5);

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
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        );
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake)); // A = X drivetrain wheels
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //  LB = Reset field-centric heading
        drivetrain.registerTelemetry(logger::telemeterize);

    /* Operator controls */
        opController.start().onTrue(new InstantCommand(() -> coralMode = !coralMode)); //Coral Mode Toggle
        opController.povDown().onTrue( //Decrease lvl
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 1) 
                        scoringLevel = 1;
                }
            )
        );
        opController.povUp().onTrue( //Increase lvl
            new InstantCommand(
                () -> {
                    scoringLevel += 1;
                    if (scoringLevel > 4)
                        scoringLevel = 4;
                }
            )
        );

        Trigger coralModeTrigger = new Trigger(() -> coralMode);
        Trigger scoringLevel1    = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2    = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3    = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4    = new Trigger(() -> scoringLevel == 4);

        // intake / outtake coral
        opController.rightBumper().onTrue(
            acquireCoral
            .andThen(indexCoral)
            .andThen(new SetWristPosition(wristSubsystem, 10))
        );

        opController.leftBumper().whileTrue(outTakeCoral);

        // moving to coral levels
        // we need to create a new HomeSystem command each time
        // because otherwise the same command is bound to multiple triggers, 
        // and that causes code to crash
        opController.a()
            .and(scoringLevel1)
            .onTrue(moveCoraLevel1)
            .onFalse(new HomeSystem(wristSubsystem, elevatorSubsystem)
        );

        opController.a()
            .and(scoringLevel2)
            .onTrue(moveCoraLevel2)
            .onFalse(new HomeSystem(wristSubsystem, elevatorSubsystem)
        );

        opController.a()
            .and(scoringLevel3)
            .onTrue(moveCoraLevel3)
            .onFalse(new HomeSystem(wristSubsystem, elevatorSubsystem)
        );

        opController.a()
            .and(scoringLevel4)
            .onTrue(moveCoraLevel4)
            .onFalse(new HomeSystem(wristSubsystem, elevatorSubsystem)
        );

    /* Technician controls */
        // elevator
        techController.start().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2));
        techController.back().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2));

        // wrist
        // techController.povDown().whileTrue(new RunWrist(wristSubsystem, -0.2));
        // techController.povUp().whileTrue(new RunWrist(wristSubsystem, 0.2));

        // gripper
        // techController.leftBumper().whileTrue(new RunRoller(gripperSubsystem, 0.2));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.getSelected();
        return new InstantCommand();
    }
}
