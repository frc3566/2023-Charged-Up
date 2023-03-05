package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmPosition extends CommandBase {    
    Arm arm;
    WPI_CANCoder CANCoder;
    double encoderMin = 0, encoderMax = 90;
    double tarAngle;
    boolean isOpenLoop;

    private CANSparkMax motor;
    private PIDController pidController;


    public ArmPosition(Arm arm, double tarAngle) {
        this.arm = arm;

        motor = new CANSparkMax(Constants.ArmConstants.PULLEY_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(30);
        motor.setSecondaryCurrentLimit(20);
        CANCoder = new WPI_CANCoder(Constants.ArmConstants.CANCODER_ID, "rio");
        arm.setBrake(true);
        pidController = new PIDController(0.1, 0, 0);
        pidController.setTolerance(Constants.ArmConstants.TOLERANCE);

        addRequirements(arm);

    }

    @Override
    public void execute() {
        double power = -pidController.calculate(CANCoder.getPosition(), tarAngle);
        if(power != 0 && isOpenLoop == false){
            motor.set(power);
        }             

        if(CANCoder.getPosition() < encoderMin){
            if(arm.getPower() > 0){
                arm.setPower(0);
            }
        }
        if(CANCoder.getPosition() > encoderMax){
            if(arm.getPower() < 0){
                arm.setPower(0);
            }
        }
    }
}