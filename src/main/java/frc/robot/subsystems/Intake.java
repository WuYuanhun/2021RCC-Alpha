// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.CanId;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

  private static final Logger logger = Logger.getLogger("frc.subsystems.intake");
  
  /** Creates a new Intake. */
  private double kintakeSpeed;
  private VictorSPX m_intake_motor = new VictorSPX(CanId.MOTOR_INTAKE);

  public Intake() {
    kintakeSpeed = 0.8;

    m_intake_motor.setNeutralMode(NeutralMode.Coast);
    m_intake_motor.setInverted(true);
  
    SmartDashboard.putNumber("Intake Speed", kintakeSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double intakeSpeed = SmartDashboard.getNumber("Hopper Speed", 0);
    if (intakeSpeed != kintakeSpeed) {
      setSpeed(intakeSpeed);
    }
  }

  private void setSpeed(double newSpeed){
    kintakeSpeed = newSpeed;
  }

  public void intakeStart(){
    m_intake_motor.set(ControlMode.PercentOutput, kintakeSpeed);
  }

  public void intakeStop(){
    m_intake_motor.set(ControlMode.PercentOutput, 0);
  }

}
