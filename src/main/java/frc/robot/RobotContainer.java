/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import java.util.logging.Logger;

import com.ctre.phoenix.ParamEnum;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PneuStatus;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.TriggerToBoolean;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static Logger logger = Logger.getLogger("frc.RobotContainer");
  
  public final DriveSubsystem m_drive = new DriveSubsystem();
  private final PneumaticSubsystem m_pneumatic = new PneumaticSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Vision m_vision = new Vision();
  private final Feeder m_feeder = new Feeder();

  private Joystick m_leftJoy = new Joystick(0);
  private Joystick m_rightJoy = new Joystick(1);
  private XboxController m_xbox = new XboxController(2);

    private JoystickButton m_back = new JoystickButton(m_xbox, Button.kBack.value);
  private JoystickButton m_start = new JoystickButton(m_xbox, Button.kStart.value);
  private JoystickButton m_A = new JoystickButton(m_xbox, Button.kA.value);
  private JoystickButton m_B = new JoystickButton(m_xbox, Button.kB.value);
  private JoystickButton m_X = new JoystickButton(m_xbox, Button.kX.value);
  private JoystickButton m_Y = new JoystickButton(m_xbox, Button.kY.value);
  private JoystickButton m_leftXboxStick = new JoystickButton(m_xbox, Button.kStickLeft.value);
  private JoystickButton m_rightXboxStick = new JoystickButton(m_xbox,Button.kStickRight.value);
  private JoystickButton m_BumperLeft = new JoystickButton(m_xbox, Button.kBumperLeft.value);
  private JoystickButton m_BumperRight = new JoystickButton(m_xbox, Button.kBumperRight.value);
  private TriggerToBoolean m_leftXboxTrigger = new TriggerToBoolean(m_xbox, Axis.kLeftTrigger.value, 0.7);
  private TriggerToBoolean m_rightXboxTrigger = new TriggerToBoolean(m_xbox, Axis.kRightTrigger.value, 0.7);

  private JoystickButton m_leftButton1 = new JoystickButton(m_leftJoy, 1);
  private JoystickButton m_leftButton2 = new JoystickButton(m_leftJoy, 2);
  private JoystickButton m_leftButton3 = new JoystickButton(m_leftJoy, 3);
  private JoystickButton m_leftButton4 = new JoystickButton(m_leftJoy, 4);
  private JoystickButton m_leftButton5 = new JoystickButton(m_leftJoy, 5);
  private JoystickButton m_leftButton6 = new JoystickButton(m_leftJoy, 6);
  private JoystickButton m_leftButton7 = new JoystickButton(m_leftJoy, 7);
  private JoystickButton m_leftButton8 = new JoystickButton(m_leftJoy, 8);
  private JoystickButton m_leftButton9 = new JoystickButton(m_leftJoy, 9);
  private JoystickButton m_leftButton10 = new JoystickButton(m_leftJoy, 10);
  private JoystickButton m_leftButton11 = new JoystickButton(m_leftJoy, 11);
  private JoystickButton m_leftButton12 = new JoystickButton(m_leftJoy, 12);
  private JoystickButton m_leftButton13 = new JoystickButton(m_leftJoy, 13);
  private JoystickButton m_leftButton14 = new JoystickButton(m_leftJoy, 14);
  private JoystickButton m_leftButton16 = new JoystickButton(m_leftJoy, 16);

  private JoystickButton m_coButton1 = new JoystickButton(m_rightJoy,1);
  private JoystickButton m_coButton2 = new JoystickButton(m_rightJoy,2);
  private JoystickButton m_coButton3 = new JoystickButton(m_rightJoy,3);
  private JoystickButton m_coButton4 = new JoystickButton(m_rightJoy,4);
  private JoystickButton m_coButton14 = new JoystickButton(m_rightJoy,14);
  private JoystickButton m_coButton15 = new JoystickButton(m_rightJoy,15);
  private JoystickButton m_coButton16 = new JoystickButton(m_rightJoy, 16);

  private final ChangeBase changeBase = new ChangeBase(m_pneumatic);
  private final ChangeIntake changeIntake = new ChangeIntake(m_pneumatic);
  private final ChangeClimber changeClimber = new ChangeClimber(m_pneumatic);
  private final ChangeLock changeLock = new ChangeLock(m_pneumatic);
  
  private final FeederWorkCommand m_feederWork = new FeederWorkCommand(m_feeder);
  private final TurretAimCommand autoAimdCommand = new TurretAimCommand(m_turret, m_vision);
  private final IntakeWorkCommand m_intakeCommand = new IntakeWorkCommand(m_intake);

  private final ChangeBase setBaseClimbing = new ChangeBase(m_pneumatic, PneuStatus.kBaseClimb);
  private final ChangeClimber setClimberUp = new ChangeClimber(m_pneumatic, PneuStatus.kClimberUp);
  private final ChangeIntake setIntakeLock = new ChangeIntake(m_pneumatic, PneuStatus.kIntakeLock);
  private final ChangeLock setLockRelease = new ChangeLock(m_pneumatic, PneuStatus.kClimberRelease);

  private final AutoShooterFire m_fire = new AutoShooterFire(m_drive, m_turret, m_vision, m_shooter, m_hopper, m_feeder);
  public final LeaveLine m_line = new LeaveLine(m_drive);
  private final AutoCommand m_auton = new AutoCommand(m_turret, m_intake, m_pneumatic, m_drive, m_vision, m_shooter, m_hopper, m_feeder);
  // Autonomous command
  private final AutoLinearDrive m_straight = new AutoLinearDrive(m_drive, 50);
  private final AutoRotate m_rotate = new AutoRotate(m_drive, 45);

  // private final ParallelCommandGroup setClimberStatus = new ParallelCommandGroup(
  //   setBaseClimbing, 
  //   setClimberUp,

  //   setIntakeLock
  // );

  // private final SequentialCommandGroup climbPrepartion = new SequentialCommandGroup(
  //   setClimberStatus,
  //   new SleepCommand(0.8),
  //   setLockRelease
  //  );

   /* The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Shooter", m_shooter);
    SmartDashboard.putData("Pneumatic", m_pneumatic);
    SmartDashboard.putData("Turret", m_turret);
    SmartDashboard.putData("Hood", m_hood);
    SmartDashboard.putData("Drive", m_drive);
    SmartDashboard.putData("Intake", m_intake);
    SmartDashboard.putData("Hopper", m_hopper);
 
  }

  private void configureButtonBindings() {
    logger.info("configure Button Bindings!");

    m_leftButton1.whenHeld(m_intakeCommand);

    // Climber Sets
    // m_leftButton1.whenHeld(new InstantCommand(m_intake::intakeStart, m_intake)).whenReleased(new InstantCommand(m_intake::intakeStop, m_intake));

    m_leftButton1.whenPressed(new InstantCommand(m_hopper::hopperStart, m_hopper)).whenReleased(new InstantCommand(m_hopper::hopperStop, m_hopper));
    // m_leftButton3.whenPressed(new InstantCommand(m_drive::leftTrainClimb, m_drive)).whenReleased(new InstantCommand(m_drive::stopMotor, m_drive));
    // m_leftButton4.whenPressed(new InstantCommand(m_drive::rightTrainClimb, m_drive)).whenReleased(new InstantCommand(m_drive::stopMotor, m_drive));

    m_leftButton5.whenPressed(new ChangeClimber(m_pneumatic, PneuStatus.kClimberUp));
    m_leftButton13.whenPressed(new ChangeClimber(m_pneumatic));
    m_leftButton12.whenPressed(new ChangeLock(m_pneumatic, PneuStatus.kClimberRelease));
    m_leftButton16.whenPressed(new ChangeLock(m_pneumatic, PneuStatus.kClimberLock));
    m_leftButton8.whenPressed(new ChangeBase(m_pneumatic));
    m_leftButton14.whenPressed(changeIntake);
    
    m_coButton1.whenPressed(new InstantCommand(m_hopper::hopperStart, m_hopper)).whenReleased(new InstantCommand(m_hopper::hopperStop,m_hopper));
    
    m_coButton14.whenHeld(m_feederWork);
    m_coButton15.whenHeld(new InstantCommand(m_feeder::feederReverseWork, m_feeder)).whenReleased(new InstantCommand(m_feeder::feederStop, m_feeder));
    m_coButton16.whenHeld(new InstantCommand(m_hopper::hopperReverseStart, m_hopper)).whenReleased(new InstantCommand(m_hopper::hopperStop, m_hopper));
    m_coButton2.whenHeld(autoAimdCommand); 

    m_coButton3.whenHeld(new InstantCommand(m_turret::turnLeft, m_turret)).whenReleased(new InstantCommand(m_turret::stopMotor, m_turret));
    m_coButton4.whenHeld(new InstantCommand(m_turret::turnRight, m_turret)).whenReleased(new InstantCommand(m_turret::stopMotor, m_turret));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings_test() {

    logger.info("configureButtonBindings");

    m_leftButton2.whenPressed(new InstantCommand(m_hopper::hopperStart, m_hopper)).whenReleased(new InstantCommand(m_hopper::hopperStop,m_hopper));

    m_leftButton3.whenHeld(changeIntake);
    m_leftButton4.whenHeld(new InstantCommand(m_intake::intakeStart, m_intake)).whenReleased(new InstantCommand(m_intake::intakeStop, m_intake));

    m_coButton1.whenHeld(m_feederWork);
    m_coButton2.whenHeld(autoAimdCommand);
    
    m_leftButton11.whenPressed(changeBase);
    m_leftButton6.whenPressed(changeClimber);
    m_leftButton14.whenPressed(changeIntake);
    // m_leftButton2.whenPressed(new InstantCommand(m_pneumatic::changeClimberOutput, m_pneumatic));
    // m_leftButton3.whenPressed(new InstantCommand(m_pneumatic::changeLockOutput,m_pneumatic));
    
    // m_leftButton7.whenPressed(m_changeBaseCommand);

    // m_leftButton6.whenPressed(new InstantCommand(m_pneumatic::changeClimberOutput, m_pneumatic));
    
    // m_leftButton3.whenPressed(new InstantCommand(m_pneumatic::changeIntakeOutput,m_pneumatic));    
    //TODO Minus sign delete or not?
    m_coButton2.whenHeld(autoAimdCommand);// minus sign?
    // m_leftButton1.whenHeld(new InstantCommand(m_turret::turnLeft, m_turret)).whenReleased(new InstantCommand(m_turret::stopMotor, m_turret));
    // m_leftButton3.whenHeld(new InstantCommand(m_turret::turnRight, m_turret)).whenReleased(new InstantCommand(m_turret::stopMotor, m_turret));
    // m_leftButton1.whenHeld(new FeederWorkCommand(m_feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auton;
  }

  public void teleopInit() {
    logger.info("teleopInit - start compressor");
    m_vision.exitAuto();
    m_vision.setLightOff();
    m_pneumatic.CompressorBegin();
    m_pneumatic.setBaseOutput(PneuStatus.kBaseDrive);
    final Command arcadeDriveCommand = new RunCommand(() -> m_drive.ArcadeDrive(m_leftJoy.getY(), -m_leftJoy.getX()), m_drive);
    m_drive.setDefaultCommand(arcadeDriveCommand);
    SmartDashboard.putNumber("AutoAim", 1);
  }

  public void disableInit() {
    logger.info("disable init - ending compressor");
    m_pneumatic.CompressorEnd();
  }
}
