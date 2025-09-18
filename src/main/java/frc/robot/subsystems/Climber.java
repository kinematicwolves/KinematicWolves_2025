// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberProfile;

public class Climber extends SubsystemBase {
  /**Elevator Motors */
  private SparkMax motor = new SparkMax(ClimberProfile.motorID, MotorType.kBrushless);
    
  /**Elevator Sensors */
  private RelativeEncoder motorEncoder = motor.getEncoder();

  /**Elevator PID Controllers */
  private SparkClosedLoopController motorController = motor.getClosedLoopController();
    
  /**Elevator Configurations */
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
    
  /** Soft Limit Config */
  private SoftLimitConfig softLimitConfig = new SoftLimitConfig();

  /** Internal Variables */
  private double setPoint = 0;

  private Servo ServoA = new Servo(0);
  private Servo ServoB = new Servo(1);
  // private  Servo ServoA = new Servo(1);
  // private  Servo ServoB = new Servo(2);
  private  double close = 0.2;
  private  double open = 0.7;
  private  boolean funnelDropped = false;
  

  /** Creates a new Climber. */
  public Climber() {
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,null);

    motorConfig.smartCurrentLimit(ClimberProfile.currentLimit);
    motorConfig.inverted(false); // we may need to change one later
    motorConfig.idleMode(IdleMode.kBrake);
    
    /**Software Limits */
    softLimitConfig.apply(motorConfig.softLimit);
    motorConfig.softLimit.reverseSoftLimitEnabled(false);
    motorConfig.softLimit.forwardSoftLimitEnabled(false);
    motorConfig.softLimit.reverseSoftLimit(0);
    motorConfig.softLimit.forwardSoftLimit(ClimberProfile.fwdSoftLimit);
    // motorConfig.softLimit.forwardSoftLimit(ClimberProfile.rvsSoftLimit);

    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ClimberProfile.kP, ClimberProfile.kI, ClimberProfile.kD);

    motor.configure(motorConfig, null, PersistMode.kPersistParameters);
  }
  
  /**
   * returns the value of the motor encoder in rotations
   * @return double, motor rotations
   */
  public double getPosition(){
    return motorEncoder.getPosition();
  }

  /**
   * Sets the Elevator position set point
   * @param targetSetPoint, double, units of motor rotations
   */
  public void setPosition(double targetSetPoint){
    setPoint = targetSetPoint;
    motorController.setReference(setPoint, ControlType.kPosition);
  }
  
  /**
   * Sets output to lift motors.
   * @param speed double, percentage output
   */
  public void setSpeed(double speed){
     motor.set(speed);
  }

  /**
   * Tells us if it between the upper and lower position 
   * @return true if between upper and lower limit, else
   */
  public boolean atPosition(){
    double lowerLimit = setPoint - ClimberProfile.encoderSetPointError;
    double UpperLimit = setPoint + ClimberProfile.encoderSetPointError;
    
    if ((getPosition() >= lowerLimit) && (getPosition()<= UpperLimit)) {
      return true;
    }
    else {
      return false;
    }
  }


  public void dropFunnel(){
    // ServoA.set(0.12);
    // ServoB.set(0.12);
    ServoA.setPosition(close);
    ServoB.setPosition(close);
     funnelDropped = true;
  }

  public void closefunnel(){
    ServoA.setPosition(open);
    ServoB.setPosition(open);
    //ServoB.set(0.50);
    funnelDropped = false;
  } 

  public boolean isFunnelDropped() {
    return funnelDropped;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber encoder position", getPosition());
    SmartDashboard.putNumber("Climber set point", setPoint);
    SmartDashboard.putBoolean("Climber At Pose", atPosition());
    SmartDashboard.putBoolean("Funnel Dropped", isFunnelDropped());
    SmartDashboard.putNumber("ServoAPose", ServoA.getPosition());
    SmartDashboard.putNumber("ServoBPose", ServoB.getPosition());
  }
}
