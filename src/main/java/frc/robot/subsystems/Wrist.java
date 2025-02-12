// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private SparkMax m_wrist = new SparkMax(60, SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig wristConfig = new SparkMaxConfig();
    private SparkClosedLoopController wristController = m_wrist.getClosedLoopController();

    private double setPoint = 0;

  /** Creates a new Wrist. */
  public Wrist() {
    m_wrist.configure(wristConfig, ResetMode.kResetSafeParameters,null);
    wristConfig.smartCurrentLimit(40);
    wristConfig.inverted(false); // we may need to change this one later
    wristConfig.idleMode(IdleMode.kCoast);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.01, 0, 0);
    m_wrist.configure(wristConfig, null, PersistMode.kPersistParameters);
  }

  /**
  * Retrieves wrist absolute through bore encoder value
  * 
  * @return double, through bore position
  */
  private double getWristPos() {
    return m_wrist.getEncoder().getPosition();
  }

  /**
   * Sets motor pid controller to commanded position.
   * @param commandedPos double, wrist encoder revolutions
   */
  public void setWristPos(double commandedPos) {
    setPoint = commandedPos;
    wristController.setReference(commandedPos, ControlType.kPosition);
  }

  public boolean wristAtPos() {
    double lowerLimit = setPoint - 5;
    double upperLimit = setPoint + 5;

    if ((getWristPos() >= lowerLimit) && (getWristPos() <= upperLimit)) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Pos", getWristPos());
  }
}
