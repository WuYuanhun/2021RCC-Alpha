// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;

public class Feeder extends SubsystemBase {

  private static final Logger logger = Logger.getLogger("frc.subsystems.feeder");

  private VictorSPX m_feeder_tvictor_left = new VictorSPX(CanId.MOTOR_FEEDER_LEFT);
  private VictorSPX m_feeder_tvictor_right = new VictorSPX(CanId.MOTOR_FEEDER_RIGHT);

  private Joystick m_rightJoy = new Joystick(1);

  double kfeederSpeed = 1;

  public Feeder() {
    m_feeder_tvictor_left.setNeutralMode(NeutralMode.Coast);
    m_feeder_tvictor_right.setNeutralMode(NeutralMode.Coast);
    m_feeder_tvictor_left.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double feederSpeed = SmartDashboard.getNumber("Hopper Speed", 0);
    // if (feederSpeed != kfeederSpeed) {
    //   setSpeed(feederSpeed);
    // }
  }

  public void feederWork() {
    m_feeder_tvictor_left.set(ControlMode.PercentOutput, kfeederSpeed);
    m_feeder_tvictor_right.set(ControlMode.PercentOutput, kfeederSpeed);
  }

  public void feederStop(){
    m_feeder_tvictor_left.set(ControlMode.PercentOutput, 0);
    m_feeder_tvictor_right.set(ControlMode.PercentOutput, 0);
  }

  public void feederReverseWork() {
    m_feeder_tvictor_left.set(ControlMode.PercentOutput, -kfeederSpeed);
    m_feeder_tvictor_right.set(ControlMode.PercentOutput, -kfeederSpeed);
  }

  public void setSpeed(double feederSpeed){
    kfeederSpeed = feederSpeed;
  }

  public boolean ready()
  {
    return SmartDashboard.getBoolean("Ready", true);
  }
}
