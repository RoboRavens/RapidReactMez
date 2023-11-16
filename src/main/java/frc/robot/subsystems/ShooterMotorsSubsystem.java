// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterMotorsSubsystem extends SubsystemBase {
  private WPI_TalonFX _shooterBackspinMotor;
  private WPI_TalonFX _shooterTopSpinMotor;
  
  public ShooterMotorsSubsystem() {
    _shooterBackspinMotor = new WPI_TalonFX(61);
    _shooterTopSpinMotor = new WPI_TalonFX(62);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorSpin() {
      _shooterBackspinMotor.set(-0.1);
      _shooterTopSpinMotor.set(0.9);


  }

  public void motorStop() {
    _shooterBackspinMotor.set(0);
    _shooterTopSpinMotor.set(0);
  }
}
