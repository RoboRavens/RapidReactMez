package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.ShooterCalibration;
import frc.util.ShooterCalibrationPair;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX _backspinMotor;
    private TalonFX _topspinMotor;
    private ShooterCalibrationPair _shot;
    private boolean _isShooting;
    private boolean _recovered;
    private double _arbitraryFeedForward = 0;
    
    public ShooterSubsystem() {
        _backspinMotor = new TalonFX(RobotMap.SHOOTER_BACKSPIN_MOTOR);
        _topspinMotor = new TalonFX(RobotMap.SHOOTER_TOPSPIN_MOTOR);
        _backspinMotor.setInverted(true);
        _topspinMotor.setInverted(false);
        _backspinMotor.setNeutralMode(NeutralMode.Coast);
        _topspinMotor.setNeutralMode(NeutralMode.Coast);
        this.setShot(Constants.TARMAC_SHOT_CALIBRATION_PAIR);
        _backspinMotor.setSelectedSensorPosition(0);
        _topspinMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        updateArbitraryFeedForward();
    }

    public void setShot(ShooterCalibrationPair shot) {
        setMotorShot(_backspinMotor, shot._backspinMotorCalibration);
        setMotorShot(_topspinMotor, shot._topspinMotorCalibration);

        this._shot = shot;
    }

    /**
     * Sets both shooter motors to the specified shot type
     * @param shot The ShooterCalibration shot to set the motors to
     */
    public void setMotorShot(TalonFX motor, ShooterCalibration shot) {
        motor.config_kF(Constants.SHOOTER_IDX, shot.kF, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kP(Constants.SHOOTER_IDX, shot.kP, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kI(Constants.SHOOTER_IDX, shot.kI, Constants.SHOOTER_TIMEOUT_MS);
        motor.config_kD(Constants.SHOOTER_IDX, shot.kD, Constants.SHOOTER_TIMEOUT_MS);
    }

    /**
    * Stops the motor (sets it to 0% power)
    */
    public void stopMotor() {
        _backspinMotor.set(ControlMode.Current, 0);
        _topspinMotor.set(ControlMode.Current, 0);
        _isShooting = false;
    }

    public void updateArbitraryFeedForward() {
        double aff = 0;
        
        switch (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceState()) {
            case OFF:
                aff += 0;
                break;
            case EJECTING:
                aff += Constants.SHOOTER_CONVEYANCE_EJECTING_ARBITRARY_FEED_FORWARD;
                break;
            case INDEXING:
                aff += Constants.SHOOTER_CONVEYANCE_INDEXING_ARBITRARY_FEED_FORWARD;
                break;
            case INTAKING:
                aff += Constants.SHOOTER_CONVEYANCE_INTAKING_ARBITRARY_FEED_FORWARD;
                break;
        }

        _arbitraryFeedForward = aff;
    }

    /**
    * Starts the motor with the set shot type
    */
    public void startMotor() {
        _recovered = false;

        double backspinTargetMotorRPM =  (_shot._backspinMotorCalibration.targetRPM / Constants.BACKSPIN_GEAR_RATIO);
        double backspinTargetVelocity = backspinTargetMotorRPM * Constants.TALON_RPM_TO_VELOCITY;

        double topspinTargetMotorRPM =  (_shot._topspinMotorCalibration.targetRPM / Constants.TOPSPIN_GEAR_RATIO);
        double topspinTargetVelocity = topspinTargetMotorRPM * Constants.TALON_RPM_TO_VELOCITY;

        // updateShooterTuningSmartDashboard(backspinTargetMotorRPM, topspinTargetMotorRPM, backspinTargetVelocity, topspinTargetVelocity);

        _backspinMotor.set(ControlMode.Velocity, backspinTargetVelocity);
        _topspinMotor.set(ControlMode.Velocity, topspinTargetVelocity);

        _backspinMotor.set(ControlMode.Velocity, backspinTargetVelocity, DemandType.ArbitraryFeedForward, _arbitraryFeedForward);
        _topspinMotor.set(ControlMode.Velocity, topspinTargetVelocity, DemandType.ArbitraryFeedForward, _arbitraryFeedForward);

        _isShooting = true;
    }
}
