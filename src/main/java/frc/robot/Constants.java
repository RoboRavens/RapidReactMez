package frc.robot;

import frc.util.ShooterCalibration;
import frc.util.ShooterCalibrationPair;

public final class Constants {
    public static final double JOYSTICK_DEADBAND = .1; // TODO: Adjust this value based on your joysticks
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 3;
    public static final double CONVEYANCE_TWO_NORMAL_SPEED = 0.25;
    public static final double CONVEYANCE_TWO_STOP = 0;

    public static final double TARMAC_BACKSPIN_RPM = 2225;
    public static final double TARMAC_BACKSPIN_KF = 0.0449 * 1.08;
    public static final double TARMAC_BACKSPIN_KP = .350;//0.21;
    public static final double TARMAC_BACKSPIN_KI = 0;
    public static final double TARMAC_BACKSPIN_KD = 4.0;
    public static final double TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_BACKSPIN_KF * 12;

    public static final double TARMAC_TOPSPIN_RPM = 2225;
    public static final double TARMAC_TOPSPIN_KF = 0.046 * 1.08;
    public static final double TARMAC_TOPSPIN_KP = .35;//0.21;
    public static final double TARMAC_TOPSPIN_KI = 0;
    public static final double TARMAC_TOPSPIN_KD = 4.0;
    public static final double TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_TOPSPIN_KF * 12;

    public static final ShooterCalibration TARMAC_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Tarmac Shot", TARMAC_BACKSPIN_RPM, TARMAC_BACKSPIN_KF, TARMAC_BACKSPIN_KP, TARMAC_BACKSPIN_KI, TARMAC_BACKSPIN_KD, TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibration TARMAC_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Tarmac Alt Shot", TARMAC_TOPSPIN_RPM, TARMAC_TOPSPIN_KF, TARMAC_TOPSPIN_KP, TARMAC_TOPSPIN_KI, TARMAC_TOPSPIN_KD, TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibrationPair TARMAC_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Tarmac Shot", TARMAC_SHOT_BACKSPIN_CALIBRATION, TARMAC_SHOT_TOPSPIN_CALIBRATION);

    //SHOOTER
    public static final int SHOOTER_IDX = 0;
    public static final int SHOOTER_TIMEOUT_MS = 100;

    public static final double BACKSPIN_GEAR_RATIO = 16.0 / 36.0;
    public static final double TOPSPIN_GEAR_RATIO = 16.0 / 24.0;

    // TALONFX
    public static final double TALONFX_TICKS_PER_REVOLUTION = 2048;
    public static final double TALON_TPS_TO_RPM = 600;
    public static final double TALON_VELOCITY_TO_RPM = TALON_TPS_TO_RPM / TALONFX_TICKS_PER_REVOLUTION;
    public static final double TALON_RPM_TO_VELOCITY = 1 / TALON_VELOCITY_TO_RPM;

    //CONVEYANCE ONE
    public static final double CONVEYANCE_ONE_FULL_SPEED_REVERSE = 1.0;
    public static final double CONVEYANCE_ONE_FULL_SPEED = -1; // Was -75, meant to be .75, but not changing functionality without testing.
    public static final double CONVEYANCE_ONE_INDEX_SPEED = -.25;
    public static final double CONVEYANCE_ONE_NORMAL_REVERSE_SPEED = -.25;
    public static final double CONVEYANCE_ONE_STOP = 0;
    
    public static final double SHOOTER_CONVEYANCE_EJECTING_ARBITRARY_FEED_FORWARD = 0.01;
    public static final double SHOOTER_CONVEYANCE_INDEXING_ARBITRARY_FEED_FORWARD = 0.01;
    public static final double SHOOTER_CONVEYANCE_INTAKING_ARBITRARY_FEED_FORWARD = 0.01;


    public static final double CONVEYANCE_TWO_FEEDER_SPEED = 1.0;
    public static final double CONVEYANCE_TWO_FULL_SPEED = .25;


}
