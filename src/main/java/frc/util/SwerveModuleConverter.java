package frc.util;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleConverter {
    /**
     * Converts a SwerveModule object into a SwerveModuleState object.
     * @param module - a SwerveModule object
     * @param angleOffsetInDegrees - added to the SwerveModule's steer angle when constructing the SwerveModuleState.
     * Pass in 0 for no offset on the module's current angle
     * @return A SwerveModuleState object
     */
    public static SwerveModuleState ToSwerveModuleState(SwerveModule module, double angleOffsetInDegrees) {
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle() + Math.toRadians(angleOffsetInDegrees)));
    }
}