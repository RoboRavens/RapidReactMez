// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.FeederCollectCommand;
import frc.robot.commands.FeederStopCommand;
import frc.robot.commands.ShooterStartCommand;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public static XboxController m_controller = new XboxController(0);
  public static DriverStation.Alliance allianceColor = Alliance.Invalid;

  public static ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final FeederSubsystem FEEDER_SUBSYSTEM = new FeederSubsystem(); 
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();

  public static final DrivetrainDefaultCommand DRIVETRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final FeederCollectCommand FEEDER_COLLECT_COMMAND = new FeederCollectCommand();
  public static final FeederStopCommand FEEDER_STOP_COMMAND = new FeederStopCommand();
  public static final ShooterStartCommand SHOOTER_START_COMMAND = new ShooterStartCommand();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    setDriverStationData();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(DRIVETRAIN_DEFAULT_COMMAND);
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.stopMotor(), SHOOTER_SUBSYSTEM));
    FEEDER_SUBSYSTEM.setDefaultCommand(FEEDER_STOP_COMMAND);
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> CONVEYANCE_SUBSYSTEM.motorStop(), CONVEYANCE_SUBSYSTEM));
    
    // // Reduces drivetrain speed when pressed
    // new Trigger(() -> m_controller.getRightTriggerAxis() > 0)
    // .whileTrue(new InstantCommand(() -> new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.cutPower())))
    // .onFalse(new InstantCommand(() -> new InstantCommand(() -> DRIVETRAIN_SUBSYSTEM.stopCutPower())));
    
    // Run the conveyance
    new Trigger(() -> m_controller.getAButton()).whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> CONVEYANCE_SUBSYSTEM.setConveyanceIndexCargoForward(), CONVEYANCE_SUBSYSTEM),
      FEEDER_COLLECT_COMMAND
    ));

    // Rev the shooter
    new Trigger(() -> m_controller.getLeftTriggerAxis() > 0).whileTrue(SHOOTER_START_COMMAND);

    // Shoot
    new Trigger(() -> m_controller.getRightTriggerAxis() > 0).whileTrue(new RunCommand(() -> FEEDER_SUBSYSTEM.forceShoot(), FEEDER_SUBSYSTEM));
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    setDriverStationData();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    setDriverStationData();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

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
