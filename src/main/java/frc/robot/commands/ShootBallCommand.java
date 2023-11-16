// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterMotorsSubsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class ShootBallCommand extends CommandBase {
  private WPI_TalonSRX _feederWheelMotor;
  private WPI_TalonFX _shooterBackspinMotor;
  private WPI_TalonFX _shooterTopSpinMotor;
  private FeederSubsystem feederSubsystem = Robot.feederSubsystem;
  private ShooterMotorsSubsystem shooterMotors = Robot.shooterMotors; 

  public ShootBallCommand() {
    addRequirements(feederSubsystem, shooterMotors);
    _feederWheelMotor = new WPI_TalonSRX(43);
    _shooterBackspinMotor = new WPI_TalonFX(61);
    _shooterTopSpinMotor = new WPI_TalonFX(62);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _feederWheelMotor.set(0.3);
    _shooterBackspinMotor.set(-0.1);
    _shooterTopSpinMotor.set(0.9);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    _feederWheelMotor.set(0);
    _shooterBackspinMotor.set(0);
    _shooterTopSpinMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
