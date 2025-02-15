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
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RunRoller;
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
    // private final CommandXboxController techController = new CommandXboxController(2);
    
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
    private final SetElevatorPosition elevatorHome        = new SetElevatorPosition(elevatorSubsystem, 0);
    private final SetElevatorPosition elevatorShallowCage = new SetElevatorPosition(elevatorSubsystem, 142.5);
    private final SetElevatorPosition elevatorAlgaeLvl1   = new SetElevatorPosition(elevatorSubsystem, -81.5);
    private final SetElevatorPosition elevatorAlgaeLvl2   = new SetElevatorPosition(elevatorSubsystem, -18.5);
    private final SetElevatorPosition elevatorScoreAlgae  = new SetElevatorPosition(elevatorSubsystem, -91.5);

    // Wrist
    private final SetWristPosition wristHome            = new SetWristPosition(wristSubsystem, 0);
    private final SetWristPosition wristCoralAquiredPos = new SetWristPosition(wristSubsystem, 0);
    private final SetWristPosition wristCoralLvl1       = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl2       = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl3       = new SetWristPosition(wristSubsystem, 4);
    private final SetWristPosition wristCoralLvl4       = new SetWristPosition(wristSubsystem, 10);
    private final SetWristPosition wristAlgaeLvl1       = new SetWristPosition(wristSubsystem, 49);
    private final SetWristPosition wristAlgaeLvl2       = new SetWristPosition(wristSubsystem, 30);
    private final SetWristPosition wristScoreAlgae      = new SetWristPosition(wristSubsystem, 60);
    
    // Gripper
    private final RunRoller intakeCoral      = new RunRoller(gripperSubsystem, 0.3);
    private final RunRoller intakeAlgae      = new RunRoller(gripperSubsystem, 0.3);
    private final RunRoller outTakeAlgae     = new RunRoller(gripperSubsystem, -0.5);
    private final RunRoller outTakeCoralFast = new RunRoller(gripperSubsystem, -0.7);
    private final RunRoller outTakeCoral     = new RunRoller(gripperSubsystem, -0.1);

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
        opController.start().onTrue(
            new InstantCommand(
                () -> {
                    coralMode = !coralMode;
                    SmartDashboard.putBoolean("Coral Mode Toggled:", coralMode); // Back = Coral/Algae mode toggle
                }
            )
        );
        opController.povDown().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 1) 
                        scoringLevel = 1;
                    System.out.println("Scoring level now set to: " + scoringLevel); // Down dpad = Decrease scoring level
                }
            )
        );
        opController.povUp().onTrue(
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
        Trigger algaeModeTrigger = new Trigger(() -> !coralMode);
        Trigger scoringLevel0    = new Trigger(() -> scoringLevel == 0);
        Trigger scoringLevel1    = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2    = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3    = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4    = new Trigger(() -> scoringLevel == 4);

        // // coralModeTrigger.and(scoringLevel1.and(
        //     opController.y().onTrue(wristAlgaeLvl1.andThen(elevatorAlgaeLvl1));
        // // coralModeTrigger.and(scoringLevel2.and(
        //     opController.b().onTrue(wristAlgaeLvl2.andThen(elevatorAlgaeLvl2));
        // opController.leftStick().onTrue(wristScoreAlgae.andThen(elevatorScoreAlgae));
        // opController.rightStick().onTrue(wristCoralLvl4.andThen(elevatorCoralLv4));

        // algaeModeTrigger.and(scoringLevel0.and(opController.y().onTrue(elevatorScoreAlgae.andThen(wristScoreAlgae))));
        // algaeModeTrigger.and(scoringLevel1.and(opController.y().onTrue(elevatorAlgaeLvl1.andThen(wristAlgaeLvl1).andThen(intakeAlgae))));
        // algaeModeTrigger.and(scoringLevel2.and(opController.y().onTrue(elevatorAlgaeLvl2.andThen(wristAlgaeLvl2).andThen(intakeAlgae))));


        opController.rightBumper().whileTrue(outTakeCoral);
        opController.leftBumper().whileTrue(intakeCoral);
        opController.leftTrigger().whileTrue(intakeAlgae);
        opController.rightTrigger().whileTrue(outTakeAlgae);
        opController.povDown().onTrue(wristCoralLvl1.andThen(elevatorCoralLv1));

        opController.a().onTrue(elevatorHome.andThen(wristHome));

        opController.povLeft().onTrue(wristCoralLvl2.andThen(elevatorCoralLv2));
        opController.povRight().onTrue(wristCoralLvl3.andThen(elevatorCoralLv3));
        opController.povUp().onTrue(wristCoralLvl4.andThen(elevatorCoralLv4));//whileTrue(outTakeCoralFast);

        /* Technician controls */
        opController.start().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2));
        opController.back().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2));

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
