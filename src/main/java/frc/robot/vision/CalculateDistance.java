package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// TODO Move constants to constant file?
// TODO Possibly change units. Everything is currently in meters.

public class CalculateDistance {

    public double toReflectiveTape() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        
        // TODO fill this out
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;
        
        // TODO fill this out
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeight = 0.0;
        
        // distance from the target to the floor
        double goalHeight = 0.56;
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
    }

}
