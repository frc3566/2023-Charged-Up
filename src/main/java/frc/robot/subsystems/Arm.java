package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
    private CANSparkMax wench;
    private boolean on = false;

    public Arm() {
        wench = new CANSparkMax(Constants.ArmConstants.wenchID, MotorType.kBrushless);
    }

    public void run() {
        if (on) {
            wench.stopMotor();
            on = false;
        } else {
            wench.set(0.2);
            on = true;
        }
    }

}