package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TelescopingArm extends SubsystemBase{
    private CANSparkMax arm = new CANSparkMax(30, MotorType.kBrushless);
    private RelativeEncoder encoder;
    private boolean isExtended;
    private SparkMaxPIDController armPID;

    private double armZero;

    private double maxPos, minPos;
    public TelescopingArm(){
        arm.setInverted(false);
        arm.setClosedLoopRampRate(0.3);
        arm.setIdleMode(IdleMode.kBrake);
        encoder = arm.getEncoder();
        encoder.setVelocityConversionFactor(1);
        isExtended = false;
    }

    public void setPower(double power){
        arm.set(power);
    }
    public void extend(){
        if(isExtended){
        return;
        }

        double tar = encoder.getPosition() + 180;

        armPID.setReference(tar, ControlType.kPosition);

        isExtended = true;
    }
    public void fullyExtend(){
        if(isExtended){
            return;
        }
    
            double tar = maxPos;
    
            armPID.setReference(tar, ControlType.kPosition);
    
            isExtended = true;
    }
    public void contract(){
        if(!isExtended){
          return;
        }
    
        double tar = encoder.getPosition() - 180;

        armPID.setReference(tar, ControlType.kPosition);
    
        isExtended = false;
    }
    public void setZero(){
        armZero = encoder.getPosition();
    }
    public void setMinPos(double minP){
        minPos = minP;
    }
    public void setMaxPos(double maxP){
        maxPos = maxP;
    }
    public double getMinPos(){
        return minPos;
    }
    public double getMaxPos(){
        return maxPos;
    }
    public double getPos(){
        return encoder.getPosition();
    }
    @Override
    public void periodic() {

        if(encoder.getPosition() <= armZero && arm.get() < 0){
            arm.set(0);
        }
        if(encoder.getPosition() >= armZero + 180 && arm.get() > 0){
            arm.set(0);
        }
        System.out.println("position: " + encoder.getPosition());
    }
}