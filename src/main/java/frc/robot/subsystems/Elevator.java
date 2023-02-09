package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase{
    private CANSparkMax lift1;
    private CANSparkMax lift2;
    private CANSparkMax extension1;
    private CANSparkMax extension2;

    public Elevator(){
        lift1 = new CANSparkMax(Constants.ElevatorConstants.lift1ID, MotorType.kBrushless);
        lift2 = new CANSparkMax(Constants.ElevatorConstants.lift2ID, MotorType.kBrushless);
        extension1 = new CANSparkMax(Constants.ElevatorConstants.extension1ID, MotorType.kBrushless);
        extension2 = new CANSparkMax(Constants.ElevatorConstants.extension2ID, MotorType.kBrushless);
    }
    
    public void run() {
    }

    public void ElevatorUp() {
        lift1.set(0.2);
        lift2.set(0.2);
    }

    public void ElevatorDown() {
        lift1.set(-0.2);
        lift2.set(-0.2);
    }

    public void ElevatorOff() {
        lift1.stopMotor();
        lift2.stopMotor();
    }

    public void ElevatorExtend() {
        extension1.set(0.2);
        extension2.set(0.2);
    }
    public void ElevatorContract() {
        extension1.set(-0.2);
        extension2.set(-0.2);
    }

    public void ElevatorExtensionOff() {
        extension1.stopMotor();
        extension2.stopMotor();
    }
    
}
