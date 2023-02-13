// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;  

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final double CAMERA_HEIGHT_METERS = 0.9;
  private final double TARGET_HEIGHT_METERS = 0.96;
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(10);

  PhotonCamera camera = new PhotonCamera("photonvision_reflectivetape");

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
  }

  public double[] getDistToReflectiveTape() {
    var result = camera.getLatestResult();
    if (result.getBestTarget() == null) {
      double[] info = { -1.0 };
      return info;
    }
    double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    double[] info = { range };
    return info;
  }

  public double[] getYawToReflectiveTape() {
    var result = camera.getLatestResult();
    if (result.getBestTarget() == null) {
      double[] info = { -1.0 };
      return info;
    }
    double yaw = result.getBestTarget().getYaw();
    double[] info = { yaw };
    return info;
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

/**
 * 
 * LOCALIZATION STUFF
 * 
 * // Data format (botpose - metres & degrees): x, y, z, roll, pitch, yaw
  public void subscribeToRobotPosition() {
    NetworkTable limelight1 = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTable limelight2 = NetworkTableInstance.getDefault().getTable("limelight2");
    botpose_1 = limelight1.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    // botpose_2 = limelight2.getDoubleArrayTopic("botpose").subscribe(new double[] {});;
  }

  public double[] getRobotPosition() {
    // FIXME temp code that handles only one limelight for testing
    double[] botpose_1_value = botpose_1.get();
    // double[] botpose_2_value = botpose_2.get();
    // if (botpose_1_value.length == 0 && botpose_2_value.length == 0) {
    //   // TODO fallback system for when apriltag vision fails (nice to have)
    //   // no apriltags are visible, just stick to last estimate (or maybe use other source of data?) some kind of fallback?
    //   return null;
    // } else if (botpose_1_value.length == 0) {
    //   return botpose_2_value;
    // } else if (botpose_2_value.length == 0) {
    //   return botpose_1_value;
    // } else {
    //   if (botpose_1_value == botpose_2_value) { return botpose_1_value; }
    //   double[] avg_botpose_value = {};
    //   for (int i = 0; i < botpose_1_value.length; i++) {
    //     avg_botpose_value[i] = (botpose_1_value[i] + botpose_2_value[i]) / 2;
    //   }
    //   return avg_botpose_value;
    // }
    return botpose_1_value;
  }

  public double getPositionVariation(int i, int testDurationSecs) {
    long startTime = System.currentTimeMillis();
    double maxPosValue = 0.0;
    double minPosValue = 100000.0;
    while (startTime != startTime + testDurationSecs * 1000) {
      maxPosValue = Math.max(getRobotPosition()[i], maxPosValue);
      minPosValue = Math.min(getRobotPosition()[i], minPosValue);
    }
    return maxPosValue - minPosValue;
  }
 */