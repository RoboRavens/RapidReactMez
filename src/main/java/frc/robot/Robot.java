// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.ShootBallCommand;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.ConveyanceTwoSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterMotorsSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static XboxController m_controller = new XboxController(0);
  public static ConveyanceSubsystem conveyanceSubsystem = new ConveyanceSubsystem();
  public static ConveyanceTwoSubsystem conveyanceTwoSubsystem = new ConveyanceTwoSubsystem();
  public static FeederSubsystem feederSubsystem = new FeederSubsystem();
  public static ShooterMotorsSubsystem shooterMotors = new ShooterMotorsSubsystem();
  public static DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM;
  public static DriverStation.Alliance allianceColor = Alliance.Invalid;
  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();

  public Robot() {
    JoystickButton button1 = new JoystickButton(m_controller, 1);
    button1.onTrue(new ShootBallCommand());
  }

  @Override
  public void robotPeriodic() {
    setDriverStationData();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Reduces drivetrain speed when pressed
    new Trigger(() -> m_controller.getRightTriggerAxis() > 0) // TODO: check this axis (does it trigger when the right trigger is pressed?)
    .whileTrue(new InstantCommand(() -> new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.cutPower())))
    .onFalse(new InstantCommand(() -> new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.stopCutPower())));

    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    setDriverStationData();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
 
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    setDriverStationData();
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
  // Due to the manner in which the robot connects to the driver station,
  // which differs between the shop and match play,
  // this method needs to called both periodically AND in the auto/tele init methods.
  private void setDriverStationData() {
    allianceColor = DriverStation.getAlliance();
  }
}
