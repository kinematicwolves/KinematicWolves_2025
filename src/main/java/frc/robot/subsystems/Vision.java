// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.LimelightHelpers;

public class Vision extends SubsystemBase {
  // objects to facilitate position estimates from limelight 
  private Debouncer alignmentFilter = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  // The pipeline the limelight should use
  private int reefPipeline = 0;

  private double tx_setpoint = 1.0;
  private double tx_setpointNegativeTolerence = tx_setpoint - 1;
  private double tx_setpointPositiceTolerence = tx_setpoint + 1;

  /* Creates a new Vision. */
  public Vision() {
    setPipeline(reefPipeline);
  }

  private void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex("limelight", pipeline);
}

private double getTx() {
  return LimelightHelpers.getTX("limelight");
}

private double getTa() {
  return LimelightHelpers.getTA("limelight");
}

public boolean xOvershot() {
  if (getTx() > tx_setpointPositiceTolerence) {
    return true;
  }
  else {
    return false;
  }
}

public boolean xUnderShot() {
  if (getTx() < tx_setpointNegativeTolerence) {
    return true;
  }
  else {
    return false;
  }
}

public boolean xAtSetpoint() {
  if ((getTx() >= tx_setpointNegativeTolerence) && (getTx() <= tx_setpointPositiceTolerence)){
    return alignmentFilter.calculate(true);
  }
  else {
    return false;
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("getTX", getTx());
    SmartDashboard.putBoolean("Robot Aligned with Reef", xAtSetpoint());
  }
}
