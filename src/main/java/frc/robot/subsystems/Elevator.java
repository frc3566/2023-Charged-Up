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
    private double fullLength = 82;
    private double extension;

    public Elevator(){
        leadMotor = new CANSparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        leadMotor.setInverted(false);
        leadMotor.setSmartCurrentLimit(20);

        followMotor = new CANSparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        followMotor.follow(leadMotor,true);
        followMotor.setSmartCurrentLimit(20);

        encoder = leadMotor.getEncoder();
        encoder.setVelocityConversionFactor(1);

        PIDController = leadMotor.getPIDController();
        PIDController.setP(0.2); //TODO tune these
        PIDController.setI(0);
        PIDController.setD(0.01);

        setBrake(true);
        this.setZero();
    }
    
    public void periodic() {
        extension = (encoder.getPosition()-encoderZero) / fullLength;
        if(extension > encoderZero + fullLength && leadMotor.get() > 0){
            leadMotor.set(0);
        }
        // if(extension < encoderZero && leadMotor.get() < 0){
        //     leadMotor.set(0);
        // }
        // System.out.println("Elevator Encoder: " +  encoder.getPosition());
        // setExtendtionPercentage(0.0);
        // System.out.println("Encoder Zero: " + encoderZero);
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

    public void setExtension(double factor) {
        //percent is between 0 and 1;
        PIDController.setReference(encoderZero + fullLength * factor, ControlType.kPosition);
        extension = factor;
    }

    public double getExtension(){
        return extension;
    }

    public void off() {
        leadMotor.stopMotor();
        followMotor.stopMotor();
    }

    public double getLength(){
        return 5 + 50 * getExtension();
    }

    public double getPower(){
        return leadMotor.get();
    }
    
}
