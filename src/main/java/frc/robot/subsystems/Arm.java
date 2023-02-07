package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
    private CANSparkMax boomWench;
    private CANSparkMax telescopingWench;
    private boolean on = false;

    public Arm() {
        boomWench = new CANSparkMax(Constants.ArmConstants.boomWenchID, MotorType.kBrushless);
    }

    public void run() {
        if (on) {
            boomWench.stopMotor();
            on = false;
        } else {
            boomWench.set(0.2);
            on = true;
        }
    }

}