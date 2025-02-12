// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.commands.RunRoller;
import frc.robot.commands.SetWristPosition;
import frc.robot.commands.acquireCoral;
    

public class RobotContainer {
    /* Swerve */
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    // private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
    //     .withDeadband(MaxSpeed * 0.1)
    //     .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //     .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
// 
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* Controllers */
    // private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);
    // private final CommandXboxController techController = new CommandXboxController(2);
    
    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    
   private final Gripper gripperSubsystem = new Gripper();

    /* Commands */
    private final acquireCoral intakeCoral = new acquireCoral(gripperSubsystem, 1);
    private final RunRoller intakeAlgae = new RunRoller(gripperSubsystem, 0.5);
    private final RunRoller outTakeAlgae = new RunRoller(gripperSubsystem, -0.5);
    private final RunRoller outTakeCoral = new RunRoller(gripperSubsystem, -1);

    private final SetWristPosition wristHome = new SetWristPosition(gripperSubsystem, 0);
    private final SetWristPosition wristScoreAlgae = new SetWristPosition(gripperSubsystem, 20);

       

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Test");
        // SmartDashboard.putData("Auto Mode", autoChooser);
// 
        configureBindings();
    }

    private void configureBindings() {
        /* Driver controls */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(
                // () -> fieldCentricDrive
        //             .withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // reset the field-centric heading on left bumper press
        // driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* Operator controls */
        opController.rightBumper().whileTrue(intakeAlgae);
        opController.leftBumper().whileTrue(outTakeAlgae);
        opController.x().whileTrue(intakeCoral);
        opController.y().whileTrue(outTakeCoral);

        opController.a().onTrue(wristHome);
        opController.b().onTrue(wristScoreAlgae);

        // controller.button.case(command)

        /* Technition controls */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // techController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // techController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // techController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // techController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* Other */
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return null;
    }
}
