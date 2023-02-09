package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Pulley extends SubsystemBase {
    private CANSparkMax boomWench;
    private CANSparkMax telescopingWench;
    private boolean onB = false;
    private boolean onT = false;

    public Pulley() {
        boomWench = new CANSparkMax(Constants.ArmConstants.boomWenchID, MotorType.kBrushless);
        telescopingWench = new CANSparkMax(Constants.ArmConstants.telescopingWenchID, MotorType.kBrushless);
    }

    public void run() {
    }

    public void boomWenchUp() {
        boomWench.set(0.2);
    }

    public void boomWenchDown() {
        boomWench.set(-0.2);
    }

    public void boomWenchOff() {
        boomWench.stopMotor();
    }

    public void telescopingWenchOut() {
        telescopingWench.set(-0.2);
    }
    public void telescopingWenchIn() {
        telescopingWench.set(0.2);
    }

    public void telescopingWenchOff() {
        telescopingWench.stopMotor();
    }
}