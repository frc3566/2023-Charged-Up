// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {}

  public double distToReflectiveTape() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    double vertical_targetOffsetAngle = ty.getDouble(0.0);
    double horizontal_targetOffsetAngle = tx.getDouble(0.0);
    
    // TODO fill this out
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;
    
    // TODO fill this out
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeight = 0.0;
    
    // distance from the target to the floor
    double goalHeight = 0.56;
    
    double vertical_angleToGoalDegrees = limelightMountAngleDegrees + vertical_targetOffsetAngle;
    double vertical_angleToGoalRadians = Math.toRadians(vertical_angleToGoalDegrees);

    double horizontal_angleToGoalDegrees = horizontal_targetOffsetAngle; 
    double horizontal_angleToGoalRadians = Math.toRadians(horizontal_angleToGoalDegrees);

    double verticalDistance = (goalHeight - limelightLensHeight) / Math.tan(vertical_angleToGoalRadians);
    double horizontalDistance = verticalDistance * Math.tan(horizontal_angleToGoalRadians);

    double euclideanDistance = Math.sqrt(verticalDistance*verticalDistance + horizontalDistance*horizontalDistance);

    return euclideanDistance;
  }

  public double toAprilTag(int n) {
    return 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}