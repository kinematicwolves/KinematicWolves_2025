// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelProfile;

public class Funnel extends SubsystemBase {
  private TalonSRX m_motorA = new TalonSRX(FunnelProfile.motorA_Id);
  private TalonSRX m_motorB = new TalonSRX(FunnelProfile.motorB_Id);

  /** Creates a new Funnel. */
  public Funnel() {
    /** Motor Resets */
    m_motorA.configFactoryDefault();
    m_motorB.configFactoryDefault();

    /** Inversion Factors */
    m_motorA.setInverted(true);
    m_motorB.setInverted(false);

    /** Current Limits */
    m_motorA.configPeakCurrentLimit(FunnelProfile.motorCurrentLimits);
    m_motorB.configPeakCurrentLimit(FunnelProfile.motorCurrentLimits);
  }

  /**
   * Sets motors on funnel at default output.
   */
  public void runMotors() {
    m_motorA.set(ControlMode.PercentOutput, FunnelProfile.motorDefaultOutput);
    m_motorB.set(ControlMode.PercentOutput, FunnelProfile.motorDefaultOutput);
  }

  /**
   * Sets funnel motors to 0% output.
   */
  public void stopMotors() {
    m_motorA.set(ControlMode.PercentOutput, 0);
    m_motorB.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Funnel motor supply current (amps)", m_motorA.getSupplyCurrent());
  }
}
