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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverProfile;
import frc.robot.Constants.ElevatorProfile;
import frc.robot.Constants.GripperProfile;
import frc.robot.Constants.VisionProfile;
import frc.robot.Constants.WristProfile;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.HomeSystemAlgae;
import frc.robot.commands.HomeSystemCoral;
import frc.robot.commands.IndexCoral;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveToLevel;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SetRollerSpeed;
import frc.robot.commands.SetWristPosition;
import frc.robot.commands.SetWristSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    /* Swerve Speeds */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(DriverProfile.kRotationMagnitude).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // 10% dead band
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    
    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(DriverProfile.driverPortNum);
    private final CommandXboxController opController = new CommandXboxController(DriverProfile.operatorPortNum);
    private final CommandXboxController techController = new CommandXboxController(DriverProfile.technicianPortNum);
    
    /* Driver station choosers */
    private final SendableChooser<Command> autoChooser;
    
    /* Robot states */
    public boolean coralMode = true;
    public int scoringLevel = 1;

    /**Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevatorSubsystem = new Elevator();
    private final Gripper gripperSubsystem = new Gripper();
    private final Wrist wristSubsystem = new Wrist();
    private final Vision vision = new Vision();

    /* Commands */
    // Swerve
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Wist and Elevator
    private final MoveToLevel moveCoralLevel1 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl1Pos,  WristProfile.coralLvl1Pos);
    private final MoveToLevel moveCoralLevel2 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl2Pos,   WristProfile.coralLvl2Pos);
    private final MoveToLevel moveCoralLevel3 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl3Pos,  WristProfile.coralLvl3Pos);
    private final MoveToLevel moveCoralLevel4 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl4Pos, WristProfile.coralLvl4Pos);
    
    private final MoveToLevel  moveAlgaeLevel1 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.algaeLvl1Pos, WristProfile.algaeLvl1Pos);
    private final MoveToLevel  moveAlgaeLevel2 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.algaeLvl2Pos, WristProfile.algaeLvl2Pos);

    private final MoveToLevel elevatorShallowCage = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.shallowCagePos, WristProfile.shallowCagePos);

    // Gripper
    private final AcquireCoral   acquireCoral = new AcquireCoral(gripperSubsystem, GripperProfile.acquireCoralSpeed);
    private final IndexCoral     indexCoral   = new IndexCoral(gripperSubsystem, GripperProfile.indexCoralSpeed);
    private final SetRollerSpeed outTake      = new SetRollerSpeed(gripperSubsystem, GripperProfile.outTakeSpeed);

    private final IntakeAlgae intakeAlgae     = new IntakeAlgae(gripperSubsystem, GripperProfile.intakeAlgaeSpeed, GripperProfile.holdAlgaeOutput);

    public RobotContainer() {
        configureBindings();
        // Named Commands for pathplanner
        NamedCommands.registerCommand("homeSystemCoral", new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("moveCoralLevel1", moveCoralLevel1);
        NamedCommands.registerCommand("moveCoralLevel2", moveCoralLevel2);
        NamedCommands.registerCommand("moveCoralLevel3", moveCoralLevel3);
        NamedCommands.registerCommand("moveCoralLevel4", moveCoralLevel4);
        NamedCommands.registerCommand("outTakeCoral", new ParallelDeadlineGroup(
                                                           new WaitCommand(1), 
                                                           new SetRollerSpeed(gripperSubsystem, 0.5)
                                                           ));
        NamedCommands.registerCommand("intakeCoral", new AcquireCoral(gripperSubsystem, 0.1)
                                                          .andThen(new IndexCoral(gripperSubsystem, 0.1)
                                                          ));

        /* Auto Selector */
        autoChooser = AutoBuilder.buildAutoChooser("LeftWall2");
        SmartDashboard.putData("Auto Mode", autoChooser);
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
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ) 
        );

        driveController.rightTrigger().whileTrue( // A = X drivetrain wheels
            drivetrain.applyRequest(() -> brake)
        ); 

        driveController.y().onTrue( //  LB = Reset field-centric heading
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // Reef auto align
        driveController.rightBumper().debounce(0.2).whileTrue(drivetrain.applyRequest(
            () -> robotCentric
                .withRotationalRate(vision.getRightReefTx(VisionProfile.frontLimelight)/VisionProfile.frontProportionalTx)
                .withVelocityX(-driveController.getLeftY() * 0.8)
                .withVelocityY(-driveController.getLeftX() * 0.5)
                ));

        driveController.leftBumper().debounce(0.2).whileTrue(
            drivetrain.applyRequest(
                () -> robotCentric
                .withRotationalRate(vision.getLeftReefTx(VisionProfile.frontLimelight)/VisionProfile.frontProportionalTx)
                .withVelocityX(-driveController.getLeftY() * 0.8)
                .withVelocityY(-driveController.getLeftX() * 0.5)
                ));
    
        driveController.rightBumper().and(driveController.leftBumper()).debounce(0.3).whileTrue(drivetrain.applyRequest(
            () -> robotCentric
                .withRotationalRate(vision.getCenterReefTx(VisionProfile.frontLimelight)/VisionProfile.frontProportionalTx)
                .withVelocityX(-driveController.getLeftY() * 0.8)
                .withVelocityY(-driveController.getLeftX() * 0.5)
                ));

        drivetrain.registerTelemetry(logger::telemeterize);


    /* Operator controls */
        opController.start().onTrue(new InstantCommand(() -> coralMode = !coralMode)); //Coral Mode Toggle
        opController.povDown().onTrue( //Decrease lvl
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 0) 
                        scoringLevel = 0;
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
        Trigger scoringLevel0    = new Trigger(() -> scoringLevel == 0);
        Trigger scoringLevel1    = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2    = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3    = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4    = new Trigger(() -> scoringLevel == 4);

        // intake / outtake coral
        opController.x()
            .and(coralModeTrigger)
            .onTrue(new ParallelRaceGroup(
                acquireCoral
                .andThen(indexCoral),
                new WaitCommand(5))); // ensures this command will terminate if it runs longer than 5 seconds.

        opController.rightTrigger()
            .whileTrue(outTake);
    
        opController.x()
            .and(coralModeTrigger.negate())
            .whileTrue(intakeAlgae);
        

        // moving to coral levels
        opController.leftTrigger()
            .and(scoringLevel1)
            .and(coralModeTrigger)
            .onTrue(moveCoralLevel1)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger()
            .and(scoringLevel2)
            .and(coralModeTrigger)
            .onTrue(moveCoralLevel2)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger()
            .and(scoringLevel3)
            .and(coralModeTrigger)
            .onTrue(moveCoralLevel3)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger()
            .and(scoringLevel4)
            .and(coralModeTrigger)
            .onTrue(moveCoralLevel4)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger() // Shallow cage climb in coral mode lvl 0
            .and(scoringLevel0)
            .and(coralModeTrigger)
            .onTrue(elevatorShallowCage)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));    

        // moving to algae levels
        opController.leftTrigger()
            .and(scoringLevel1)
            .and(coralModeTrigger.negate())
            .onTrue(moveAlgaeLevel1)
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger()
            .and(scoringLevel2)
            .and(coralModeTrigger.negate())
            .onTrue(moveAlgaeLevel2)
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));

        opController.leftTrigger()
            .and(scoringLevel0)
            .and(coralModeTrigger.negate())
            .onTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeScorePos)
            .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeScorePos)))
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem)
        );

    /* Technician controls */
        // elevator
        techController.start().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2));
        techController.back().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2));

        // wrist
        techController.povDown().whileTrue(new SetWristSpeed(wristSubsystem, -0.2));
        techController.povUp().whileTrue(new SetWristSpeed(wristSubsystem, 0.2));

        // gripper
        techController.leftBumper().whileTrue(new SetRollerSpeed(gripperSubsystem, 0.3));
        techController.rightBumper().whileTrue(new SetRollerSpeed(gripperSubsystem, 0.1));
        techController.b().whileTrue(new SetRollerSpeed(gripperSubsystem, -0.3));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
