// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;

public class Hood extends SubsystemBase {
  private static final Logger logger = Logger.getLogger("frc.subsystems.hood");

  private CANSparkMax m_hood_motor = new CANSparkMax(CanId.MOTOR_HOOD, MotorType.kBrushless);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder; // 0 ~ -9.5 (raw)
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private static Joystick joy = new Joystick(0);

  NetworkTableEntry hoodPositionEntry;
  NetworkTableEntry hoodTimestampEntry;

  public Hood() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("hood");
    hoodPositionEntry = table.getEntry("hood");
    hoodTimestampEntry = table.getEntry("timestamp");

    m_hood_motor.restoreFactoryDefaults();
    m_hood_motor.setIdleMode(IdleMode.kBrake);

    m_pidController = m_hood_motor.getPIDController();
    m_encoder = m_hood_motor.getEncoder();
    m_encoder.setPosition(0);

    kP = 0.3;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
  
    SmartDashboard.putNumber("Hood P Gain", kP);
    SmartDashboard.putNumber("Hood I Gain", kI);
    SmartDashboard.putNumber("Hood D Gain", kD);
    SmartDashboard.putNumber("Hood I Zone", kIz);
    SmartDashboard.putNumber("Hood Feed Forward", kFF);
  }

  @Override
  public void periodic() {
    hoodPositionEntry.setDouble(m_encoder.getPosition());
    hoodTimestampEntry.setDouble(Timer.getFPGATimestamp());
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Hood P Gain", 0);
    double i = SmartDashboard.getNumber("Hood I Gain", 0);
    double d = SmartDashboard.getNumber("Hood D Gain", 0);
    double iz = SmartDashboard.getNumber("Hood I Zone", 0);
    double ff = SmartDashboard.getNumber("Hood Feed Forward", 0);
    double max = SmartDashboard.getNumber("Hood Max Output", 0);
    double min = SmartDashboard.getNumber("Hood Min Output", 0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    
    double v = joy.getThrottle();
    double setpoint = (v + 1.0) / 2.0 * (-9.5);

    // logger.info(String.format("setpoint: %.2f", setpoint));


    // if(m_pidController.setReference( setpoint, ControlType.kPosition) == CANError.kOk) {
    //   if(m_hood_motor.get() != 0.0) {
    //   logger.fine(String.format("HOOD PID report: %.2f", m_hood_motor.get()));
    //   }
    // }
    // else
    // {
    //   logger.fine("chushi chushi chushi");
    // }
    // SmartDashboard.putNumber("SetPoint", -9.5);
    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }

  public void setPosition(double targetPosition) {
    
  }
}
