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
import frc.robot.commands.CloseFunnel;
import frc.robot.commands.DropFunnel;
import frc.robot.commands.HomeSystemAlgae;
import frc.robot.commands.HomeSystemCoral;
import frc.robot.commands.IndexCoral;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveToLevel;
import frc.robot.commands.MoveToLevelAlgae;
import frc.robot.commands.RunClimberOpenLoop;
import frc.robot.commands.RunElevatorOpenLoop;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.commands.SetRollerSpeed;
import frc.robot.commands.SetWristPosition;
import frc.robot.commands.SetWristSpeed;
import frc.robot.commands.LightingCommands.ReefAlignmentLIghtshow;
import frc.robot.commands.LightingCommands.SetDisableLightShow;
import frc.robot.commands.LightingCommands.SetTeleOpLightShow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    /* Swerve Speeds */
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double maxAngularRate = RotationsPerSecond.of(DriverProfile.kRotationMagnitude).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1) // 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    /* Controllers */
    private final CommandXboxController driveController = new CommandXboxController(DriverProfile.driverPortNum);
    private final CommandXboxController opController = new CommandXboxController(DriverProfile.operatorPortNum);
    private final CommandXboxController techController = new CommandXboxController(DriverProfile.technicianPortNum);

    /* Driver Station Choosers */
    private final SendableChooser<Command> autoChooser;

    /* Robot States */
    public boolean coralMode = true;
    public int scoringLevel = 1;

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevatorSubsystem = new Elevator();
    private final Gripper gripperSubsystem = new Gripper();
    private final Wrist wristSubsystem = new Wrist();
    private final Lighting lighting = new Lighting();
    private final Climber climber = new Climber();
    private final Vision vision = new Vision();

    /* Commands */
    // Swerve
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private final Telemetry logger = new Telemetry(maxSpeed);

    // Wrist and Elevator
    private final MoveToLevel moveCoralLevel1 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl1Pos, WristProfile.coralLvl1Pos);
    private final MoveToLevel moveCoralLevel2 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl2Pos, WristProfile.coralLvl2Pos);
    private final MoveToLevel moveCoralLevel3 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl3Pos, WristProfile.coralLvl3Pos);
    private final MoveToLevel moveCoralLevel4 = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.coralLvl4Pos, WristProfile.coralLvl4Pos);

    private final MoveToLevelAlgae moveAlgaeLevel1 = new MoveToLevelAlgae(wristSubsystem, elevatorSubsystem, ElevatorProfile.algaeLvl1Pos, WristProfile.algaeLvl1Pos);
    private final MoveToLevelAlgae moveAlgaeLevel2 = new MoveToLevelAlgae(wristSubsystem, elevatorSubsystem, ElevatorProfile.algaeLvl2Pos, WristProfile.algaeLvl2Pos);

    // climber 
    private final DropFunnel dropFunnel = new DropFunnel(climber);
    private final CloseFunnel closeFunnel = new CloseFunnel(climber);
    // private final MoveToLevel elevatorShallowCage = new MoveToLevel(wristSubsystem, elevatorSubsystem, ElevatorProfile.shallowCagePos, WristProfile.shallowCagePos);

    // Gripper
    private final AcquireCoral acquireCoral = new AcquireCoral(gripperSubsystem, GripperProfile.acquireCoralSpeed);
    private final IndexCoral indexCoral = new IndexCoral(gripperSubsystem, GripperProfile.indexCoralSpeed);
    private final SetRollerSpeed outTake = new SetRollerSpeed(gripperSubsystem, GripperProfile.outTakeSpeed);

    private final IntakeAlgae intakeAlgae = new IntakeAlgae(gripperSubsystem, GripperProfile.intakeAlgaeSpeed, GripperProfile.holdAlgaeOutput);


    public RobotContainer() {
        // Configure control bindings (button mappings and input handlers)
        configureBindings();
    
        /* Named Commands for PathPlanner */
        // Register various robot commands to be used in autonomous routines
        NamedCommands.registerCommand("homeSystemCoral", new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("homeSystemAlgae", new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("moveCoralLevel1", moveCoralLevel1);
        NamedCommands.registerCommand("moveCoralLevel2", moveCoralLevel2);
        NamedCommands.registerCommand("moveCoralLevel3", moveCoralLevel3);
        NamedCommands.registerCommand("moveCoralLevel4", moveCoralLevel4);
        NamedCommands.registerCommand("algaeLevel1", new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeLvl1Pos)
                                                                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeLvl1Pos, 0)));
        NamedCommands.registerCommand("algaeLevel2", new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeLvl2Pos)
                                                                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeLvl2Pos, 0)));
        NamedCommands.registerCommand("algaeLevel0", new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeScorePos)
                                                                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeScorePos, 0)));
        NamedCommands.registerCommand("toggleToAlgaeMode", new InstantCommand(() -> coralMode = false)); // setting it to false is better, since you want to guarantee it's in algae mode
    
        // Command to outtake coral: Runs the roller at 50% speed for 1 second
        NamedCommands.registerCommand("outTakeCoral", new ParallelDeadlineGroup(
            new WaitCommand(0.3), 
            new SetRollerSpeed(gripperSubsystem, 0.5)
        ));
    
        // Command to intake coral: First acquires at 10% speed, then indexes at 10% speed
        NamedCommands.registerCommand("intakeCoral", new AcquireCoral(gripperSubsystem, GripperProfile.acquireCoralSpeed)
            .andThen(new IndexCoral(gripperSubsystem, GripperProfile.indexCoralSpeed))
        );

        NamedCommands.registerCommand("intakeAlgae", new ParallelDeadlineGroup(
            new WaitCommand(5),
            new IntakeAlgae(gripperSubsystem, GripperProfile.intakeAlgaeSpeed, GripperProfile.holdAlgaeOutput)
        ));

        // Command to outtake coral: Runs the roller at 50% speed for 1 second
        NamedCommands.registerCommand("outTakeAlgae", new ParallelDeadlineGroup(
             new WaitCommand(0.3),
             new SetRollerSpeed(gripperSubsystem, 0.5)
             ));
            
    
        /* Autonomous Selector */
        // Builds the autonomous chooser with a default starting option
        autoChooser = AutoBuilder.buildAutoChooser("LeftWall3");
    
        // Displays the auto mode selection on the SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
    /* Driver Controls */
        // Note that X is defined as forward according to WPILib convention, and Y is defined as left.
        // Default drivetrain command for field-centric control.
        // if (elevatorSubsystem.getPosition() > ElevatorProfile.maxElevatorCG) { // Slows down when elevator is above center of gravity threshold position.
        //         drivetrain.applyRequest(
        //             () -> fieldCentricDrive
        //                 .withVelocityX((-driveController.getLeftY() * maxSpeed) * DriverProfile.x_SlowMultiplier) // Forward/backward movement
        //                 .withVelocityY((-driveController.getLeftX() * maxSpeed) * DriverProfile.y_SlowMultiplier) // Left/right movement
        //                 .withRotationalRate((-driveController.getRightX() * maxAngularRate) * DriverProfile.rx_SlowMultiplier) // Rotational movement
        //     );
        // }
        // else {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(elevatorSubsystem.getPosition() > ElevatorProfile.maxElevatorCG ? (-driveController.getLeftY() * maxSpeed) * DriverProfile.y_SlowMultiplier : -driveController.getLeftY() * maxSpeed) // Forward/backward movement
                    .withVelocityY(elevatorSubsystem.getPosition() > ElevatorProfile.maxElevatorCG ? (-driveController.getLeftX() * maxSpeed) * DriverProfile.y_SlowMultiplier : -driveController.getLeftX() * maxSpeed) // Left/right movement
                    .withRotationalRate(elevatorSubsystem.getPosition() > ElevatorProfile.maxElevatorCG ? (-driveController.getRightX() * maxAngularRate) * DriverProfile.rx_SlowMultiplier : -driveController.getRightX() * maxAngularRate) // Rotational movement
            )
        );
        // }

        // driveController.leftTrigger().whileTrue(drivetrain.applyRequest(
        //     () -> fieldCentricDrive
        //         .withVelocityX(-driveController.getLeftY() * DriverProfile.y_slowMode)
        //         .withVelocityY(-driveController.getLeftX() * DriverProfile.x_slowMode)
        //         .withRotationalRate(-driveController.getRightX() * DriverProfile.rx_slowMode)
        //     )
        // );
    
        // Engage brake mode when the right trigger is held
        driveController.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
    
        // Reset field-centric heading when 'Y' is pressed
        driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        /* Reef Auto-Alignment */
        // Right bumper aligns to the right reef using vision
        driveController.rightBumper().debounce(0.1).whileTrue(
            drivetrain.applyRequest(() -> robotCentric
                .withRotationalRate(0)
                .withVelocityX(-driveController.getLeftY() * DriverProfile.y_AlignmentMultiplier) // Reduced speed for fine adjustments
                .withVelocityY(vision.getRightReefTx(VisionProfile.elevatorLimelight) / 18)//driveController.getLeftX() * DriverProfile.x_AlignmentMultiplier)
            )
        );

        driveController.rightBumper().onTrue(new ReefAlignmentLIghtshow(vision, lighting)).onFalse(new SetTeleOpLightShow(lighting, elevatorSubsystem, wristSubsystem, gripperSubsystem));
        //driveController.rightBumper().onFalse(new SetTeleOpLightShow(lighting, elevatorSubsystem, wristSubsystem, gripperSubsystem));
        
    
        // Left bumper aligns to the left reef using vision
        driveController.leftBumper().debounce(0.1).whileTrue(
            drivetrain.applyRequest(() -> robotCentric
            .withRotationalRate(0)
            .withVelocityX(-driveController.getLeftY() * DriverProfile.y_AlignmentMultiplier) // Reduced speed for fine adjustments
            .withVelocityY(vision.getLeftReefTx(VisionProfile.elevatorLimelight) / 23)//driveController.getLeftX() * DriverProfile.x_AlignmentMultiplier)
        )
        );

        driveController.leftBumper().onTrue(new ReefAlignmentLIghtshow(vision, lighting)).onFalse(new SetTeleOpLightShow(lighting, elevatorSubsystem, wristSubsystem, gripperSubsystem));
        //driveController.leftBumper().onFalse(new SetTeleOpLightShow(lighting, elevatorSubsystem, wristSubsystem, gripperSubsystem));
    
        // Both bumpers align to the center reef using vision
        driveController.rightBumper().and(driveController.leftBumper()).debounce(0.2).whileTrue(
            drivetrain.applyRequest(() -> robotCentric
                .withRotationalRate(vision.getCenterReefTx(VisionProfile.frontLimelight) / VisionProfile.reefProportionalTx)
                .withVelocityX(-driveController.getLeftY() * DriverProfile.y_AlignmentMultiplier) // Reduced speed for fine adjustments
                .withVelocityY(-driveController.getLeftX() * DriverProfile.x_AlignmentMultiplier)
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    
    /* Operator Controls */
        // Decrease scoring level when POV Down is pressed
        opController.povDown().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel -= 1;
                    if (scoringLevel < 0) 
                        scoringLevel = 0;
                }
            )
        );

        // Increase scoring level when POV Up is pressed
        opController.povUp().onTrue(
            new InstantCommand(
                () -> {
                    scoringLevel += 1;
                    if (scoringLevel > 4)
                        scoringLevel = 4;
                }
            )
        );

        // Toggle Coral Mode and home the appropriate system
        opController.start().onTrue(
            new InstantCommand(
                () -> {
                    coralMode = !coralMode; 
                }
            )
        );
    
        /* Triggers for Coral Mode and Scoring Levels */
        Trigger coralModeTrigger = new Trigger(() -> coralMode);
        Trigger scoringLevel0 = new Trigger(() -> scoringLevel == 0);
        Trigger scoringLevel1 = new Trigger(() -> scoringLevel == 1);
        Trigger scoringLevel2 = new Trigger(() -> scoringLevel == 2);
        Trigger scoringLevel3 = new Trigger(() -> scoringLevel == 3);
        Trigger scoringLevel4 = new Trigger(() -> scoringLevel == 4);

        coralModeTrigger.onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
        coralModeTrigger.onTrue(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
    
        /* Intake / Outtake */
        // Intake coral when 'X' is pressed in Coral Mode
        opController.x()
            .and(coralModeTrigger)
            .onTrue(new ParallelRaceGroup(
                acquireCoral.andThen(indexCoral),
                new WaitCommand(5) // Stop command after 5 seconds
            ));
        
        // Reverse Intake Backup Button
        opController.b()
            .and(coralModeTrigger)
            .onTrue(
                new InstantCommand(() -> gripperSubsystem.setRollerSpeed(GripperProfile.backUpIntakeSpeed)
                ))
            .onFalse(new InstantCommand(() -> gripperSubsystem.setRollerSpeed(0)));

        // Intake algae when 'X' is pressed in Algae Mode
        opController.x()
            .and(coralModeTrigger.negate())
            .whileTrue(intakeAlgae);
    
        // Outtake coral when right trigger is held
        opController.rightTrigger().whileTrue(outTake);
    
        /* Move to Coral Scoring Levels */
        opController.leftTrigger().and(scoringLevel1).and(coralModeTrigger)
            .onTrue(moveCoralLevel1)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
    
        opController.leftTrigger().and(scoringLevel2).and(coralModeTrigger)
            .onTrue(moveCoralLevel2)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
    
        opController.leftTrigger().and(scoringLevel3).and(coralModeTrigger)
            .onTrue(moveCoralLevel3)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
    
        opController.leftTrigger().and(scoringLevel4).and(coralModeTrigger)
            .onTrue(moveCoralLevel4)
            .onFalse(new HomeSystemCoral(wristSubsystem, elevatorSubsystem));
    
        /* Move to Algae Scoring Levels */
        opController.leftTrigger().and(scoringLevel1).and(coralModeTrigger.negate())
            .onTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeLvl1Pos)
                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeLvl1Pos, 0)))
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
    
        opController.leftTrigger().and(scoringLevel2).and(coralModeTrigger.negate())
            .onTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeLvl2Pos)
                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeLvl2Pos, 0)))
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
    
        // Algae scoring position (Level 0)
        opController.leftTrigger().and(scoringLevel0).and(coralModeTrigger.negate())
            .onTrue(new SetElevatorPosition(elevatorSubsystem, ElevatorProfile.algaeScorePos)
                .andThen(new SetWristPosition(wristSubsystem, WristProfile.algaeScorePos, 0)))
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
        
        opController.leftTrigger().and(coralModeTrigger.negate())
            .onFalse(new HomeSystemAlgae(wristSubsystem, elevatorSubsystem));
        
        // Climbing
        // opController.y().whileTrue(dropFunnel);
        // opController.a().whileTrue(closeFunnel);
        opController.y().whileTrue(new SetWristSpeed(wristSubsystem, -0.2)); // Move down
        opController.a().whileTrue(new SetWristSpeed(wristSubsystem, 0.2)); // Move up
    
    /* Technician Controls */
        // Manual elevator control
        techController.start().whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.2)); // Move up
        techController.back().whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.2)); // Move down
        techController.a().whileTrue(new RunElevatorOpenLoop(elevatorSubsystem, techController.getHID()));
    
        // Manual wrist control
        techController.povDown().whileTrue(new SetWristSpeed(wristSubsystem, -0.2)); // Move down
        techController.povUp().whileTrue(new SetWristSpeed(wristSubsystem, 0.2)); // Move up
    
        // Manual gripper control
        techController.leftBumper().whileTrue(new SetRollerSpeed(gripperSubsystem, 0.3)); // Intake
        techController.rightBumper().whileTrue(new SetRollerSpeed(gripperSubsystem, 0.1)); // Hold
        techController.b().whileTrue(new SetRollerSpeed(gripperSubsystem, -0.3)); // Outtake

        techController.rightTrigger().onTrue(new RunClimberOpenLoop(climber, techController));
    }    

    public Command getDisableInitCommand() {
        return new SetDisableLightShow(lighting);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected().alongWith(new InstantCommand(() -> lighting.setLvl0LightShow()));
    }

    public Command getTeleOpCommand() {
        return new SetTeleOpLightShow(lighting, elevatorSubsystem, wristSubsystem, gripperSubsystem);
    }
}
