package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
    public CANSparkMax motor;
    private PIDController pidController;
    WPI_CANCoder CANCoder;
    double encoderMin = 0, encoderMax = 90;
    double tarAngle;
    public boolean isOpenLoop;

    double FF, kP, kD;
    // what are these for?
    // private boolean onB = false;
    // private boolean onT = false;

    public Arm() {
        isOpenLoop = true;
        motor = new CANSparkMax(Constants.ArmConstants.PULLEY_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(30);
        motor.setSecondaryCurrentLimit(20);
        CANCoder = new WPI_CANCoder(Constants.ArmConstants.CANCODER_ID, "rio");
        setBrake(true);

        // tarAngle = CANCoder.getPosition();

        // tarAngle = 0;

        // pidController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
        pidController = new PIDController(0.2, 0, 0);
        pidController.setTolerance(Constants.ArmConstants.TOLERANCE);
    }

    public void periodic() {
        double power = -pidController.calculate(CANCoder.getPosition(), tarAngle);
        if(power != 0 && isOpenLoop == false){
            motor.set(power);
        }             

        if(CANCoder.getPosition() < encoderMin){
            if(getPower() > 0){
                setPower(0);
            }
        }
        if(CANCoder.getPosition() > encoderMax){
            if(getPower() < 0){
                setPower(0);
            }
        }
    }

    public double getPower(){
        return motor.get();
    }

    public void setBrake(boolean isBrake){
        IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
        motor.setIdleMode(sparkMode);
    }

    public void setPower(double power) {
        isOpenLoop = true;
        motor.set(power);
        tarAngle = CANCoder.getPosition();
    }

    public void setVoltage(double voltage){
        isOpenLoop = true;
        motor.setVoltage(voltage);
        tarAngle = CANCoder.getPosition();
    }

    public void setAngle(double tar){
        tarAngle = tar;
        isOpenLoop = false;
    }

    public void setZero(){
        this.CANCoder.setPosition(0);
    }

    public double getCANCoderPosition() {
        return CANCoder.getPosition();
    }

    public void off() {
        motor.stopMotor();
    }

}