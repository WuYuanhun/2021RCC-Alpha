// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Hopper extends SubsystemBase {

  private static final Logger logger = Logger.getLogger("frc.subsystems.hopper");


  private double khopperSpeed;
  private Joystick m_rightJoy = new Joystick(1);

  private VictorSPX m_hopper_motor = new VictorSPX(CanId.MOTOR_HOPPER);

  public Hopper() {
    khopperSpeed = 0.5;

    m_hopper_motor.setNeutralMode(NeutralMode.Coast);

    SmartDashboard.putNumber("Hopper Speed", khopperSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double hopperSpeed = SmartDashboard.getNumber("Hopper Speed", 0);
    // if (hopperSpeed != khopperSpeed) {
    //   setSpeed(hopperSpeed);
    // }

  }

  private void setSpeed(double newSpeed){
    khopperSpeed = newSpeed;
  }

  public void hopperStart(){
    m_hopper_motor.set(ControlMode.PercentOutput, khopperSpeed);
  }

  public void hopperStop(){
    m_hopper_motor.set(ControlMode.PercentOutput, 0);
  }

  public void hopperReverseStart() {
    m_hopper_motor.set(ControlMode.PercentOutput, -khopperSpeed);
  }
}
