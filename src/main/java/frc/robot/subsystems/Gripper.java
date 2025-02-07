// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    // declaring all the things in the subsystem
    // wrist portion
    private SparkMax m_wrist = new SparkMax(55, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private SparkAbsoluteEncoder wristAbsoluteEncoder = m_wrist.getAbsoluteEncoder();

    // roller portion
    private TalonFX m_gripper = new TalonFX(56);
    public TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
    private TimeOfFlight m_TOF = new TimeOfFlight(57);

    // init internal variables
    // wristPosition
    private double rollerSpeed = 0;
    private double wristSetPoint = 0;
        
    /** Creates a new Gripper. */
    public Gripper() {
       // apply any motor settings
        // wrist motor
        m_wrist.configure(wristConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,null);
        wristConfig.smartCurrentLimit(40);
        wristConfig.inverted(false); // we may need to change this one later
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
        m_wrist.configure(wristConfig, null, PersistMode.kPersistParameters);

        // roller motor
        m_gripper.getConfigurator().apply(new TalonFXConfiguration());
        gripperConfig.CurrentLimits.SupplyCurrentLimit = 10;
        gripperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Untested
        gripperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_gripper.getConfigurator().refresh(gripperConfig);
    }

    /**
     * Coral is inside the gripper
     * 
     * @return true if coral is in gripper, else its false
     */
    public boolean hasCoral() {
        if (m_TOF.getRange() <= 75) // TODO: Put in constants file
            return true;
        else 
            return false;
    }

    /**
     * User sets the roller speed
     * 
     * @param speed commanded output value, -1 to 1
     */public void setRollerSpeed(double speed) {
        this.rollerSpeed = speed;
        m_gripper.set(speed);
    }
    
    /**
     * Retrieves wrist absolute through bore encoder value
     * 
     * @return double, through bore position
     */
    private double getWristAbsolutePos() {
        return wristAbsoluteEncoder.getPosition();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // smart dashboard
        SmartDashboard.putNumber("Wrist Pos", getWristAbsolutePos());
        // wrist position
        SmartDashboard.putNumber("Gripper speed", this.rollerSpeed);
    }
}