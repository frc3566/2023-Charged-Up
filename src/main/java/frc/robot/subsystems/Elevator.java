package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase{
    private CANSparkMax lift;
    private CANSparkMax extension1;
    private CANSparkMax extension2;
    private RelativeEncoder encoder;
    private SparkMaxPIDController armPID;

    public Elevator(){
        lift = new CANSparkMax(Constants.ElevatorConstants.liftID, MotorType.kBrushless);
        extension1 = new CANSparkMax(Constants.ElevatorConstants.extension1ID, MotorType.kBrushless);
        extension2 = new CANSparkMax(Constants.ElevatorConstants.extension2ID, MotorType.kBrushless);
    }
    
    public void run() {
    }

    public void ElevatorUp() {
        lift.set(0.2);
    }

    public void ElevatorDown() {
        lift.set(-0.2);
    }

    public void fullyDown(){
        armPID.setReference(Constants.ElevatorConstants.maxHeight, ControlType.kPosition);
    }

    public void fullyUp(){
        armPID.setReference(Constants.ElevatorConstants.minHeight, ControlType.kPosition);
    }

    public void ElevatorOff() {
        lift.stopMotor();
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
