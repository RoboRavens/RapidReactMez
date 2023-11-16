// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootBallCommand;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.ConveyanceTwoSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterMotorsSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static XboxController m_controller;
  public static ConveyanceSubsystem conveyanceSubsystem;
  public static ConveyanceTwoSubsystem conveyanceTwoSubsystem;
  public static FeederSubsystem feederSubsystem;
  public static ShooterMotorsSubsystem shooterMotors;

  public Robot() {
    m_controller = new XboxController(0);
    conveyanceSubsystem  = new ConveyanceSubsystem();
    conveyanceTwoSubsystem = new ConveyanceTwoSubsystem();
    feederSubsystem = new FeederSubsystem();
    shooterMotors = new ShooterMotorsSubsystem();

    JoystickButton button1 = new JoystickButton(m_controller, 1);

    button1.onTrue(new ShootBallCommand());

  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
 
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    conveyanceSubsystem.motorSpin();
    conveyanceTwoSubsystem.motorSpin();
    feederSubsystem.motorRotate();
    shooterMotors.motorSpin();

  }
 

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
