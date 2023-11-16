// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  private WPI_TalonSRX _feederWheelMotor;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    _feederWheelMotor = new WPI_TalonSRX(43);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorRotate() {
    _feederWheelMotor.set(0.3);
  }

  public void motorHalt() {
    _feederWheelMotor.set(0);
  }
}
