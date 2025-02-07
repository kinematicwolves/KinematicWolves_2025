// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    /**Elevator Motors */
    private SparkMax m_LiftA = new SparkMax(50, MotorType.kBrushless);
    private SparkMax m_LiftB = new SparkMax(51, MotorType.kBrushless);
    
    /**Elevator Sensors */
    private RelativeEncoder LiftEncoderA = m_LiftA.getEncoder();
    private RelativeEncoder liftEncoderB = m_LiftB.getEncoder();

    /**Elevator PID Controllers */
    private SparkClosedLoopController lifControllerA = m_LiftA.getClosedLoopController();
    private SparkClosedLoopController lifControllerB = m_LiftB.getClosedLoopController();
    
    /**Elevator Configurations */
    private SparkMaxConfig liftConfigA = new SparkMaxConfig();
    private SparkMaxConfig liftConfigB = new SparkMaxConfig();
    
    /** Soft Limit Config */
    private SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    /** Internal Variables */
    private double setpoint = 0;

    /** Creates a new Elevator. */
    public Elevator() {
        /**Factory Reset */
        m_LiftA.configure(liftConfigA, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,null);
        m_LiftB.configure(liftConfigB, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, null);

        /**Inversion Factors */
        liftConfigA.inverted(false);
        liftConfigB.inverted(false);

        /**Current Limits */
        liftConfigA.smartCurrentLimit(40);
        liftConfigB.smartCurrentLimit(40);

        /**Software Limits */
        softLimitConfig.apply(liftConfigA.softLimit);
        liftConfigA.softLimit.forwardSoftLimit(10);
        softLimitConfig.apply(liftConfigB.softLimit);
        liftConfigB.softLimit.forwardSoftLimit(10);

        /** Netural Modes */
        liftConfigA.idleMode(IdleMode.kBrake);
        liftConfigB.idleMode(IdleMode.kBrake);

        /** Set Position */
        liftEncoderB.setPosition(0);
        LiftEncoderA.setPosition(0);

        /**Burning Configs */
        m_LiftA.configure(liftConfigA, null, PersistMode.kPersistParameters);
        m_LiftB.configure(liftConfigB, null, PersistMode.kPersistParameters);

        //**PIDS */
        liftConfigA.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
        liftConfigB.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
    }
        
    /**
     * returns the value of the motor encoder in rotations
     * @return double, motor rotations
     */
    private double getPosition(){
        return LiftEncoderA.getPosition();
    }

    /**
     * Sets the Elevator position setpoint
     * @param targetSetPoint, double, units of motor rotations
     */
    public void setPosition(double targetSetPoint){
        setpoint = targetSetPoint;
        lifControllerA.setReference(setpoint, ControlType.kPosition);
        lifControllerB.setReference(setpoint, ControlType.kPosition);
    }

    /**
     * Tells us if it between the upper and lower position 
     * @return true if between upper and lower limit, else
     */
    public boolean atPosition(){
        double lowerLimit = setpoint - 100;
        double UpperLimit = setpoint + 100;

        if ((getPosition() >= lowerLimit) && (getPosition()<= UpperLimit)) {
        return true;
        }
        else {
        return false;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator encoder position", getPosition());
        SmartDashboard.putNumber("Elevator set point", setpoint);

    }
}
