// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private DoubleArraySubscriber botpose_1;
  private DoubleArraySubscriber botpose_2;

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    subscribeToRobotPosition();
  }

  // Data format (botpose - metres & degrees): x, y, z, roll, pitch, yaw
  public void subscribeToRobotPosition() {
    NetworkTable limelight1 = NetworkTableInstance.getDefault().getTable("limelight1");
    NetworkTable limelight2 = NetworkTableInstance.getDefault().getTable("limelight2");
    botpose_1 = limelight1.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    botpose_2 = limelight2.getDoubleArrayTopic("botpose").subscribe(new double[] {});;
  }

  public double[] getRobotPosition() {
    double[] botpose_1_value = botpose_1.get();
    double[] botpose_2_value = botpose_2.get();
    if (botpose_1_value.length == 0 && botpose_2_value.length == 0) {
      // no apriltags are visible, just stick to last estimate (or maybe use other source of data?) some kind of fallback?
      return null;
    } else if (botpose_1_value.length == 0) {
      return botpose_2_value;
    } else if (botpose_2_value.length == 0) {
      return botpose_1_value;
    } else {
      if (botpose_1_value == botpose_2_value) { return botpose_1_value; }
      double[] avg_botpose_value = {};
      for (int i = 0; i < botpose_1_value.length; i++) {
        avg_botpose_value[i] = (botpose_1_value[i] + botpose_2_value[i]) / 2;
      }
      return avg_botpose_value;
    }
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