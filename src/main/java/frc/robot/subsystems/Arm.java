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
    private CANSparkMax motor;
    private PIDController pidController;
    WPI_CANCoder CANCoder;
    double encoderMin = -10, encoderMax = 80;
    double tarAngle;
    boolean isOpenLoop;

    double FF, kP, kD;
    // what are these for?
    // private boolean onB = false;
    // private boolean onT = false;

    public Arm() {
        motor = new CANSparkMax(Constants.ArmConstants.PULLEY_MOTOR_ID, MotorType.kBrushless);
        CANCoder = new WPI_CANCoder(Constants.ArmConstants.CANCODER_ID, "rio");
        setBrake(true);

        tarAngle = CANCoder.getPosition();

        pidController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
        pidController.setTolerance(Constants.ArmConstants.TOLERANCE);
        isOpenLoop = false;
    }

    public void periodic() {
        double power = pidController.calculate(CANCoder.getPosition(), tarAngle);
        if(power != 0 && isOpenLoop == false){
            // setPower(power);
        }       

        if(CANCoder.getPosition() < encoderMin){
            if(getPower() > 0){
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
    }

    public void setVoltage(double voltage){
        isOpenLoop = true;
        motor.setVoltage(voltage);
    }

    public void setAngle(double tar){
        tarAngle = tar;
    }

    public double getCANCoderPosition() {
        return CANCoder.getPosition();
    }

    public void off() {
        motor.stopMotor();
        isOpenLoop = false;
    }

}