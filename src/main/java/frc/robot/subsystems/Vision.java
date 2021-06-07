// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTableEntry tX;
  private NetworkTableEntry tY;
  private NetworkTableEntry tV;
  private NetworkTableEntry ledMode;
  private static double m_headingError;
  private static double m_VerticalError;
  private static boolean m_getValidTarget;

  private double angleLowerBound = 35;
  private double angleUpperBound = 55;
  private double speedLowerBound = 2000;
  private double speedUpperBound = 4000;
  private double gravity = 9.81;
  
  private double target_height = 2;
  private double limelight_mount_height = 0.5;
  private double limelight_mount_angle = 20;

  public boolean inAuto = true; 

  public Vision() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ytz");
    tX = table.getEntry("tx");
    tY = table.getEntry("tY");
    tV = table.getEntry("tv");
    ledMode = table.getEntry("ledMode");

    SmartDashboard.putNumber("AutoAim", 0);
  }

  @Override
  public void periodic() {
    m_headingError = tX.getDouble(0.0);
    m_VerticalError = tY.getDouble(0.0);
    m_getValidTarget = ((int) (tV.getDouble(0))) == 1;
  }

  public class solveParameters {
    private double setSpeed;
    private double setAngle;
    private double dist;
    private double constant;

  

    public void solveParameters() {
      dist = getTargetDistance();
      constant = Math.pow(2.0 * gravity * target_height, 0.5);
      setSpeed = 3500.0;
      setAngle = 0.0;
    }



    public double getAngle() {
      setAngle = Math.asin(constant / setSpeed);
      while (setAngle < angleLowerBound || setAngle > angleUpperBound) {
        if (setAngle - angleLowerBound < 1.0) {
          setSpeed -= 100.0;
        } else if (setAngle - angleUpperBound > 1.0) {
          setSpeed += 100.0;
        }
        setAngle = Math.asin(constant / setSpeed);
      }
      return setAngle;
    }
  }

  public void exitAuto() {
    inAuto = false;
  }

  public double getTargetDistance() {
    return Math.tan(getTargetAngle() + limelight_mount_angle) * (target_height - limelight_mount_height);
  }

  public double getTargetAngle()
  {
    return 0.9 * m_headingError;
  }

  public double getVerticalAngle() {
    return 0.9 * m_VerticalError;
  }

  public double getTargetSpeed() {
    return 0.0;
  }

  public static double getHeadingError()
  {
    return m_headingError;
  }

  public static boolean getValidTarget() {
    return m_getValidTarget;
  }

  public void toggleLight()
  {
    Number mode = ledMode.getNumber(0);
    if (mode.intValue() == 0 || mode.intValue() == 3)
    {
      ledMode.setNumber(1);
    }
    else if (mode.intValue() == 1)
    {
      ledMode.setNumber(3);
    }
  }

  public int getLightValue()
  {
    return ledMode.getNumber(0).intValue();
  }

  public void setLightOn()
  {
    ledMode.setNumber(3);
  }

  public void setLightOff()
  {
    ledMode.setNumber(1);
  }
  
}
