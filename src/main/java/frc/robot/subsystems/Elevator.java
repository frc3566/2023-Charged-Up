package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase{
    private CANSparkMax leadMotor;
    private CANSparkMax followMotor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    private double encoderZero;
    private double fullLength = 1000; //TODO measure this

    public Elevator(){
        leadMotor = new CANSparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        leadMotor.setInverted(false);

        followMotor = new CANSparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        followMotor.setInverted(true);
    
        followMotor.follow(leadMotor);

        encoder = leadMotor.getEncoder();
        encoder.setVelocityConversionFactor(1);

        PIDController = leadMotor.getPIDController();
        PIDController.setP(0.3); //TODO tune these
        PIDController.setI(0);
        PIDController.setD(0.01);

        setBrake(true);
    }
    
    public void run() {
    }

    public void setPower(double power) {
        leadMotor.set(power);
        followMotor.set(power);
    }

    public void setVoltage(double voltage) {
        leadMotor.setVoltage(voltage);
        followMotor.setVoltage(voltage);
    }

    public void setBrake(boolean isBrake){
        IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
        leadMotor.setIdleMode(sparkMode);
        followMotor.setIdleMode(sparkMode);
    }

    public void setZero(){
        encoderZero = encoder.getPosition();
    }

    public void setExtendtionPercentage(double percent) {
        //percent is between 0 and 1;
        PIDController.setReference(encoderZero + fullLength * percent, ControlType.kPosition);

    }

    public void off() {
        leadMotor.stopMotor();
        followMotor.stopMotor();
    }
    
}
