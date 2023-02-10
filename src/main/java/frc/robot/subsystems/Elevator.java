package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends Arm{
    private CANSparkMax lift;
    private CANSparkMax extension1;
    private CANSparkMax extension2;
    private boolean isExtend;
    private double elevatorMax, elevatorZero;

    public Elevator(){
        lift = new CANSparkMax(Constants.ElevatorConstants.liftID, MotorType.kBrushless);
        extension1 = new CANSparkMax(Constants.ElevatorConstants.extension1ID, MotorType.kBrushless);
        extension2 = new CANSparkMax(Constants.ElevatorConstants.extension2ID, MotorType.kBrushless);
    }
    
    public void run() {
    }

    public void Up() {
        lift.set(0.2);
    }

    public void Down() {
        lift.set(-0.2);
    }

    public void Off() {
        lift.stopMotor();
    }

    public void Extend() {
        extension1.set(0.2);
        extension2.set(0.2);
    }
    public void Contract() {
        extension1.set(-0.2);
        extension2.set(-0.2);
    }

    public void fullyExtend(){
        if(isExtended){
            return;
        }
    
            double tar = elevatorMax;
    
            armPID.setReference(tar, ControlType.kPosition);
    
            isExtended = true;
    }

    public void ElevatorExtensionOff() {
        extension1.stopMotor();
        extension2.stopMotor();
    }

    public void setArmZero(double eleZero){
        elevatorZero = eleZero;
    }

    public void setArmMax(double eleMax){
        elevatorMax = eleMax;
    }
    
}