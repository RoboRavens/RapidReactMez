package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederStopCommand extends CommandBase {
    public FeederStopCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.FEEDER_SUBSYSTEM.conveyanceStop();;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.conveyanceStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
