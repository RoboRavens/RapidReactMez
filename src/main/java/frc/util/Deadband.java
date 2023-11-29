package frc.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Deadband {
    public static Double adjustValueToZero(double value, double minimumValue) {
        if (value >= minimumValue || value <= (minimumValue * -1)) {
            return value;
        }
        
        return 0.0d;
    }

    // If all 4 swerve modules have 0 velocity then stop modules from rotating
    public static void adjustRotationWhenStopped(SwerveModuleState[] desiredStates, SwerveModuleState[] hardwareStates) {
      for(int i = 0; i < 4; i++) {
        if (desiredStates[i].speedMetersPerSecond != 0) {
          return;
        }
      }

      for(int i = 0; i < 4; i++) {
        desiredStates[i].angle = hardwareStates[i].angle;
      }
    }
}