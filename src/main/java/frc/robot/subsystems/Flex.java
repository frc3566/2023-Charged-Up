package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

abstract class Flex extends SubsystemBase{

    public void Up() {
        
    }

    public void Down() {
        
    }

    public void Off() {
        lift.stopMotor();
    }

    public void Extend() {
        
    }
    public void Contract() {
        
    }

    public void ElevatorExtensionOff() {
        
    }
}