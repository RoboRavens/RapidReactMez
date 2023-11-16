// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyanceTwoSubsystem extends SubsystemBase {
  private WPI_TalonFX _conveyanceMotorTwo;
  
  public ConveyanceTwoSubsystem() {
    _conveyanceMotorTwo = new WPI_TalonFX(42);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorSpin() {
    _conveyanceMotorTwo.set(0.5);
  }

  public void motorStop() {
    _conveyanceMotorTwo.set(0);
  }
}
