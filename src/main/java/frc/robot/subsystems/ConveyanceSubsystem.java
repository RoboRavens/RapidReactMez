// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.ConveyanceState;

public class ConveyanceSubsystem extends SubsystemBase {
  private WPI_TalonFX _conveyanceMotorOne;
  private ConveyanceState _conveyanceState = ConveyanceState.OFF;

  public ConveyanceSubsystem() {
    _conveyanceMotorOne = new WPI_TalonFX(41);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ConveyanceState getConveyanceState() {
    return _conveyanceState;
  }

  public void setConveyanceEjectCargo() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_FULL_SPEED_REVERSE);
    _conveyanceState = ConveyanceState.EJECTING;
  }

  public void setConveyanceIntakeCargo() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_FULL_SPEED);
    _conveyanceState = ConveyanceState.INTAKING;
  }

  public void setConveyanceIndexCargoForward() {
    // _isIndexingFromStagingToFeeder = true;
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_INDEX_SPEED);
    _conveyanceState = ConveyanceState.INDEXING;
  }

  private void runConveyanceAtVoltage(double magnitude) {
    double voltage = magnitude * 12.0;
    this._conveyanceMotorOne.setVoltage(voltage);
  }

  public void motorSpin() {
    _conveyanceMotorOne.set(-0.5);
  }

  public void motorStop() {
    _conveyanceMotorOne.set(0);
  }
}
