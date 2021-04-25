/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Converters;
import frc.robot.Interfaces.*;

@SuppressWarnings({ "all" })
public class Shooter extends SubsystemBase implements ShooterInterface {
  private double targetSpeed;
  private double targetAngle;

  // TODO: calculate this converter's value and implement it to the shooter set
  // speed fragment
  double RPM_CONVERTER = 0;

  private final WPI_TalonFX m_tfx_shooter_left = new WPI_TalonFX(Constants.CanId.MOTOR_SHOOTER_LEFT);
  private final WPI_TalonFX m_tfx_shooter_right = new WPI_TalonFX(Constants.CanId.MOTOR_SHOOTER_RIGHT);

  private TalonFXConfiguration configWheel = new TalonFXConfiguration();

  // private final PIDController shooterPID = new
  // PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  public Shooter() {
    configShooterFX();
    m_tfx_shooter_left.setInverted(true);

    m_tfx_shooter_left.setNeutralMode(NeutralMode.Coast);
    m_tfx_shooter_right.setNeutralMode(NeutralMode.Coast);

    m_tfx_shooter_left.configAllSettings(configWheel);
    m_tfx_shooter_right.configAllSettings(configWheel);
  }

  @Override
  public void periodic() {
    configShooterFX();
  }

  public void StopMotor() {
    m_tfx_shooter_left.stopMotor();
    m_tfx_shooter_right.stopMotor();
  }

  public double getVelocity() {
    return m_tfx_shooter_left.getSelectedSensorVelocity();
  }

  public void setPower(double power) {
    m_tfx_shooter_left.set(ControlMode.PercentOutput, clamp(power));
    m_tfx_shooter_right.set(ControlMode.PercentOutput, clamp(power));
    System.out.println("Current velocity in RPM: " + RawSensorUnittoRPM(m_tfx_shooter_left.getSelectedSensorVelocity()));
  }

  public double clamp(double input) {
    if (input < 0) {
      return 0;
    } else if (input > 1) {
      return 1;
    }
    return input;
  }

  // must be in RPM
  public void setVelocity(double velocity) {
    m_tfx_shooter_left.set(ControlMode.Velocity, RPMtoRawSensorUnit(velocity));
    m_tfx_shooter_right.set(ControlMode.Velocity, RPMtoRawSensorUnit(velocity));
    System.out.println("P = " + configWheel.slot0.kP + " Output = " + m_tfx_shooter_left.getMotorOutputPercent() + " Actual = " + RawSensorUnittoRPM(m_tfx_shooter_left.getSelectedSensorVelocity()));
    targetSpeed = velocity;
  }

  public void setCurrent(double current) {
    m_tfx_shooter_left.set(ControlMode.Current, current);
    m_tfx_shooter_right.set(ControlMode.Current, current);
  }

  public void getCurrent() {
    System.out.println("LM output current = " + m_tfx_shooter_left.getOutputCurrent() + " LM supply current = " + m_tfx_shooter_left.getSupplyCurrent());
    System.out.println("RM output current = " + m_tfx_shooter_right.getOutputCurrent() + " RM supply current = " + m_tfx_shooter_right.getSupplyCurrent());
  }

  public double RPMtoRawSensorUnit(double velocity) {
    return velocity * 2048 / 600;
  }

  public double RawSensorUnittoRPM(double velocity) {
    return velocity / 2048 * 600;
  }

  public void outputRPM() {
    SmartDashboard.putNumber("RPM", RawSensorUnittoRPM(m_tfx_shooter_left.getSelectedSensorVelocity()));
  }

  // values: P 0.06,1; I 0.000125; D 0;
  private void configShooterFX() {
    if (RawSensorUnittoRPM(m_tfx_shooter_left.getSelectedSensorVelocity()) < 800) {
      configWheel.slot0.kP = 0.03;
    } else {
      configWheel.slot0.kP = 0.8;
      System.out.println("Damn!");
    }
    //TODO: get a suitable rawZone for Integration
    // the fixed suitable integral starting point is 1850 when the speed is 3000RPM
    int rawZone = 1; // 3000RPM - 1850
    m_tfx_shooter_left.config_IntegralZone(0, (int) ((0.62 * targetSpeed + 14) * 2048 / 600));
    m_tfx_shooter_right.config_IntegralZone(0, (int) ((0.62 * targetSpeed + 14) * 2048 / 600));
    configWheel.slot0.kI = 0.000125;
    configWheel.slot0.kD = 0;
    configWheel.openloopRamp = 1;
  }
}
