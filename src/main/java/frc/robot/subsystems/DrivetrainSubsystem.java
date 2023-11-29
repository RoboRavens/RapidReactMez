package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Deadband;
import frc.util.SwerveModuleConverter;

import static frc.robot.RobotMap.*;

public class DrivetrainSubsystem extends SubsystemBase {
    // Since Mk4ModuleBuilder has been deprecated, this code was taken from that folder to account for the missing Gear Ratios
    public enum GearRatio {
        L1(SdsModuleConfigurations.MK4_L1),
        L2(SdsModuleConfigurations.MK4_L2),
        L3(SdsModuleConfigurations.MK4_L3),
        L4(SdsModuleConfigurations.MK4_L4);

        private final MechanicalConfiguration configuration;

        GearRatio(MechanicalConfiguration configuration) {
            this.configuration = configuration;
        }

        public MechanicalConfiguration getConfiguration() {
            return configuration;
        }
    } 

    private Boolean _cutPower = false;
    private final SwerveDriveOdometry _odometryFromHardware;
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
    );
    
    private SwerveModuleState[] _moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    /**
     * The maximum velocity of the robot in meters per second.
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L1.getDriveReduction() *
        SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    public DrivetrainSubsystem() {
        MkModuleConfiguration moduleConfig = new MkModuleConfiguration();
        moduleConfig.setSteerCurrentLimit(30.0);
        moduleConfig.setDriveCurrentLimit(40.0);
        moduleConfig.setSteerPID(0.2, 0.0, 0.1);

        m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
        .withGearRatio(GearRatio.L1.getConfiguration())
        .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
        .build();

        m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
        .withGearRatio(GearRatio.L1.getConfiguration())
        .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
        .build();

        m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
        .withGearRatio(GearRatio.L1.getConfiguration())
        .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
        .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
        .build();

        m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
        .withGearRatio(GearRatio.L1.getConfiguration())
        .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
        .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
        .build();

        double swerveDriveDelay = 0;
        double swerveRotateDelay = 0.25;
        ((WPI_TalonFX) m_frontLeftModule.getSteerMotor()).configOpenloopRamp(swerveRotateDelay); 
        ((WPI_TalonFX) m_frontRightModule.getSteerMotor()).configOpenloopRamp(swerveRotateDelay);
        ((WPI_TalonFX) m_backLeftModule.getSteerMotor()).configOpenloopRamp(swerveRotateDelay);
        ((WPI_TalonFX) m_backRightModule.getSteerMotor()).configOpenloopRamp(swerveRotateDelay);

        ((WPI_TalonFX) m_frontLeftModule.getDriveMotor()).configOpenloopRamp(swerveDriveDelay); 
        ((WPI_TalonFX) m_frontRightModule.getDriveMotor()).configOpenloopRamp(swerveDriveDelay);
        ((WPI_TalonFX) m_backLeftModule.getDriveMotor()).configOpenloopRamp(swerveDriveDelay);
        ((WPI_TalonFX) m_backRightModule.getDriveMotor()).configOpenloopRamp(swerveDriveDelay);

        _odometryFromHardware = new SwerveDriveOdometry(
            m_kinematics, this.getGyroscopeRotation(), 
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = _moduleStates; // States and _modulestates still point to the same data
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        // Convert each of our SwerveModule objects into SwerveModuleState objects
        var statesHardware = new SwerveModuleState[4];
        statesHardware[0] = SwerveModuleConverter.ToSwerveModuleState(m_frontLeftModule, 0);
        statesHardware[1] = SwerveModuleConverter.ToSwerveModuleState(m_frontRightModule, 0);
        statesHardware[2] = SwerveModuleConverter.ToSwerveModuleState(m_backLeftModule, 0);
        statesHardware[3] = SwerveModuleConverter.ToSwerveModuleState(m_backRightModule, 0);

        // Updates the robot's field position as stored in our _odometryFromHardware object as a Pose2D
        // We can access this Pose2D object directly through _odometryFromHardware.getPoseMeters()
        // and through the provided method in this class, getPose()
        _odometryFromHardware.update(
        this.getGyroscopeRotation(), 
        new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        });

        // Stops the all four swerve modules from rotating if all have 0 velocity
        Deadband.adjustRotationWhenStopped(states, statesHardware);
        
        //
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void drive(ChassisSpeeds targetChassisSpeeds) {
        if (_cutPower) {
            targetChassisSpeeds.omegaRadiansPerSecond =  targetChassisSpeeds.omegaRadiansPerSecond * 0.5;
            targetChassisSpeeds.vxMetersPerSecond =  targetChassisSpeeds.vxMetersPerSecond * 0.5;
            targetChassisSpeeds.vyMetersPerSecond =  targetChassisSpeeds.vyMetersPerSecond * 0.5;
          }

        _moduleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
    }

    public void cutPower() {
        _cutPower = true;
    }

    public void stopCutPower() {
        _cutPower = false;
    }

    public boolean powerIsCut() {
        return _cutPower;
    }

    public Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
          // We will only get valid fused headings if the magnetometer is calibrated
          return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }
    
        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getAngle());
    }

    public Rotation2d getOdometryRotation() {
        return _odometryFromHardware.getPoseMeters().getRotation();
    }

    public Pose2d getPose() {
        return _odometryFromHardware.getPoseMeters();
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }
    
    public Rotation2d getPoseRotation() {
        return getPose().getRotation();
    }
}
