// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyanceSubsystem extends SubsystemBase {
  private WPI_TalonFX _conveyanceMotorOne;

  public ConveyanceSubsystem() {
    _conveyanceMotorOne = new WPI_TalonFX(41);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorSpin() {
    _conveyanceMotorOne.set(-0.5);
  }

  public void motorStop() {
    _conveyanceMotorOne.set(0);
  }
}
