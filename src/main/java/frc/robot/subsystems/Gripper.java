// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    private TalonFX m_gripper = new TalonFX(56);
    public TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
    private TimeOfFlight m_TOF = new TimeOfFlight(57);

    private double rollerSpeed = 0;
            
    /** Creates a new Gripper. */
    public Gripper() {
        m_gripper.getConfigurator().apply(new TalonFXConfiguration());
        gripperConfig.CurrentLimits.SupplyCurrentLimit = 10;
        gripperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gripper speed", this.rollerSpeed);
    }
}