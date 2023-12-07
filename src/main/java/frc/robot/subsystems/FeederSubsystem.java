// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {
  private WPI_TalonFX _conveyanceMotorTwo;
  private WPI_TalonSRX _feederWheelMotor;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    _conveyanceMotorTwo = new WPI_TalonFX(RobotMap.FEEDER_CONVEYANCE_MOTOR);
    _feederWheelMotor = new WPI_TalonSRX(43);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setConveyanceNormalSpeedForward() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_TWO_NORMAL_SPEED);
  }

  private void runConveyanceAtVoltage(double magnitude) {
    double voltage = magnitude * 12.0;
    this._conveyanceMotorTwo.setVoltage(voltage);
  }

  public void conveyanceStop() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_TWO_STOP);
  }

  public void feederWheelForward() {
    this.runFeederAtVoltage(Constants.CONVEYANCE_TWO_FEEDER_SPEED);
  }

  private void runFeederAtVoltage(double magnitude) {
    // _feederWheelMotor.set(ControlMode.PercentOutput, magnitude);
    
    // The better way to do this would be to update the constant values on
    // the methods that call this, but until we know it works I don't want
    // to change all the code to do that so we'll just do the conversion here.
    double voltage = magnitude * 12.0;
    this._feederWheelMotor.setVoltage(voltage);
  }

  public void forceShoot() {
    feederWheelForward();
    setConveyanceTwoMaxForward();
  }

  public void setConveyanceTwoMaxForward() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_TWO_FULL_SPEED);
  }

  ///////////////////

  public void motorRotate() {
    _feederWheelMotor.set(0.3);
  }

  public void motorHalt() {
    _feederWheelMotor.set(0);
  }
}
