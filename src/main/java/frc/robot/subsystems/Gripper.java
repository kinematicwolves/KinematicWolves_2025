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
import frc.robot.Constants.GripperProfile;

public class Gripper extends SubsystemBase {
    private TalonFX m_gripper = new TalonFX(GripperProfile.motorId);
    public TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
    private TimeOfFlight m_TOF = new TimeOfFlight(GripperProfile.tofId);

    private double rollerSpeed = 0;
            
    /** Creates a new Gripper. */
    public Gripper() {
        /* Factory Reset */
        m_gripper.getConfigurator().apply(new TalonFXConfiguration());

        /* Current Limit */
        gripperConfig.CurrentLimits.SupplyCurrentLimit = GripperProfile.currentLimit;

        /* Inversion Factor */
        gripperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        /* Idle Mode */
        gripperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Save Config */
        m_gripper.getConfigurator().refresh(gripperConfig);
    }

    /**
     * Coral is inside the gripper
     * 
     * @return true if coral is in gripper, else its false
     */
    public boolean hasCoral() {
        if (m_TOF.getRange() <= 200)
            return true;
        else 
            return false;
    }

    public boolean coralStowed() {
        if (m_TOF.getRange() <= 185)
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
        //SmartDashboard.putNumber("Gripper speed", this.rollerSpeed);
        SmartDashboard.putNumber("Gripper current", m_gripper.getStatorCurrent().getValueAsDouble());
        //SmartDashboard.putNumber("Tof distance", m_TOF.getRange());
        //SmartDashboard.putBoolean("GripperHasCorral", this.coralStowed());
    }
}