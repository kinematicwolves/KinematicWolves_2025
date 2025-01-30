// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;


public class Swerve extends SubsystemBase {
    public CommandSwerveDrivetrain swerveDrive = TunerConstants.createDrivetrain();

    private SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(TunerConstants.maxSpeed * 0.1)
        .withRotationalDeadband(TunerConstants.maxAngularRate * 0.1);

    private SwerveRequest.SwerveDriveBrake breakRequest = new SwerveRequest.SwerveDriveBrake();

    private PIDController angularController = new PIDController(
        Constants.SwerveProfile.kP,
        Constants.SwerveProfile.kI,
        Constants.SwerveProfile.kD
    );

    private boolean brake = false;
    private boolean angularControllerOn = false;
    private double velocityX = 0;
    private double velocityY = 0;
    private double velocityR = 0;

    /** Creates a new SwerveDrive. */
    public Swerve() {}

    public void setWheelBreak(boolean brake) {
        this.brake = brake;
    }

    public void setAngularControllerOnOff(boolean controllerOn) {
        this.angularControllerOn = controllerOn;
    }

    public void setVelocityX(double velocity) {
        this.velocityX = velocity;
    }

    public void setVelocityY(double velocity) {
        this.velocityY = velocity;
    }

    public void setVelocityAngular(double velocity) {
        this.velocityR = velocity;
    }

    public void resetHeading() {
        this.swerveDrive.seedFieldCentric();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      // if we do want to break
      if (this.brake) 
        this.swerveDrive.setControl(this.breakRequest);
      // if are not using the PID controller
      else
        this.fieldCentricDrive.withRotationalRate(this.velocityR * TunerConstants.maxAngularRate);

        this.fieldCentricDrive.withVelocityX(this.velocityX * TunerConstants.maxSpeed);
        this.fieldCentricDrive.withVelocityY(this.velocityY * TunerConstants.maxSpeed);

        // apply the active swerve request
        this.swerveDrive.setControl(this.fieldCentricDrive);
      }
}