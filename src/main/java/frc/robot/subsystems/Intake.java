package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motor;

  public Intake(){
    motor = new CANSparkMax(Constants.IntakeConstants.intakeID, MotorType.kBrushless);
    motor.setInverted(true);
    motor.setSmartCurrentLimit(40);
    motor.setSecondaryCurrentLimit(20);
    setBrake(true);
  }


  public void setBrake(boolean isBrake){
    IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
    motor.setIdleMode(sparkMode);
}

  public void setPower(double power) {
    motor.set(power);
  }

  public void off() {
    motor.stopMotor();
  }
  
}
