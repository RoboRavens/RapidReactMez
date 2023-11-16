package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveDriveOdometry _odometryFromHardware;
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(RobotMap.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotMap.DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
        new Translation2d(RobotMap.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotMap.DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
        new Translation2d(-RobotMap.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotMap.DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
        new Translation2d(-RobotMap.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotMap.DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
    );

    public DrivetrainSubsystem() {
        Mk4ModuleConfiguration moduleConfig = new Mk4ModuleConfiguration();
        moduleConfig.setSteerCurrentLimit(30.0);
        moduleConfig.setDriveCurrentLimit(40.0);
        // moduleConfig.setSteerPID(0.2, 0.0, 0.1);

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L1,
                RobotMap.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                RobotMap.FRONT_LEFT_MODULE_STEER_MOTOR,
                RobotMap.FRONT_LEFT_MODULE_STEER_ENCODER,
                RobotMap.FRONT_LEFT_MODULE_STEER_OFFSET // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        );

        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L1,
            RobotMap.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            RobotMap.FRONT_RIGHT_MODULE_STEER_MOTOR,
            RobotMap.FRONT_RIGHT_MODULE_STEER_ENCODER,
            RobotMap.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        // m_frontLeftModule = new Mk4SwerveModuleBuilder(moduleConfig)
        // .withGearRatio(GearRatio.L1.getConfiguration())
        // .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
        // .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR)
        // .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
        // .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
        // .build();

        // m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
        // .withGearRatio(GearRatio.L1.getConfiguration())
        // .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
        // .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR)
        // .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
        // .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
        // .build();

        // m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
        // .withGearRatio(GearRatio.L1.getConfiguration())
        // .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR)
        // .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR)
        // .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
        // .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
        // .build();

        // m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
        // .withGearRatio(GearRatio.L1.getConfiguration())
        // .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
        // .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR)
        // .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
        // .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
        // .build();

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
}
