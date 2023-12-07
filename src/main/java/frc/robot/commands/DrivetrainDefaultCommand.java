package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.util.Deadband;

public class DrivetrainDefaultCommand extends CommandBase {
    public boolean CutPower = false;

    public DrivetrainDefaultCommand() {
        System.out.println("drivetrain default constructor");
        addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
    }

    @Override
    public void initialize() {
        System.out.println("drivetrain default init");
    }

    @Override
    public void execute() {
        double controllerDirection = Robot.allianceColor == Alliance.Red ? 1 : -1;
        double x = Robot.m_controller.getRawAxis(0) * controllerDirection; // Positive x is away from your alliance wall.
        double y = Robot.m_controller.getRawAxis(1) * controllerDirection; // Positive y is to your left when standing behind the alliance wall.
        double r = Robot.m_controller.getRawAxis(4) * -1; // The angular rate of the robot.
        Rotation2d a = Robot.DRIVETRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);
        r = Deadband.adjustValueToZero(r, Constants.JOYSTICK_DEADBAND);

        x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

        var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, // x translation
            y, // y translation
            r, // rotation
            a // The angle of the robot as measured by a gyroscope.
        );

        if (x == 0 && y == 0 && r == 0) {
            Robot.DRIVETRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        } else {
            Robot.DRIVETRAIN_SUBSYSTEM.drive(targetChassisSpeeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.DRIVETRAIN_SUBSYSTEM.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
