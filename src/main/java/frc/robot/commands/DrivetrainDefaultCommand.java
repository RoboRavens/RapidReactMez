package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DrivetrainDefaultCommand extends CommandBase {
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double x = Robot.m_controller.getRawAxis(0) * -1; // Robot.JOYSTICK.getRawAxis(1); // Positive x is away from your alliance wall.
        double y = Robot.m_controller.getRawAxis(1) * -1; // Robot.JOYSTICK.getRawAxis(0); // Positive y is to your left when standing behind the alliance wall.
        double r; // The angular rate of the robot.
        Rotation2d a = Robot.DRIVE_TRAIN_SUBSYSTEM.getOdometryRotation(); // The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero when it is facing directly away from your alliance station wall.

        x = Deadband.adjustValueToZero(x, Constants.JOYSTICK_DEADBAND);
        y = Deadband.adjustValueToZero(y, Constants.JOYSTICK_DEADBAND);

        double rightJoystickInput = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX) * -1; // Robot.JOYSTICK.getRawAxis(2);
        rightJoystickInput = Deadband.adjustValueToZero(rightJoystickInput, Constants.JOYSTICK_DEADBAND);

        // SmartDashboard.putNumber("Drive Time", Timer.getFPGATimestamp());
        // SmartDashboard.putNumber("Drive X", x);
        // SmartDashboard.putNumber("Drive Y", y);

        x = x * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        y = y * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

    }
}
