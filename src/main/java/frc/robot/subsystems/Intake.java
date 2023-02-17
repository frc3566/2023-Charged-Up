package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax intake;

  public Intake(){
    intake = new CANSparkMax(Constants.IntakeConstants.intakeID, MotorType.kBrushless);
  }

  public void IntakeIn() {
    intake.set(0.2);
}

  public void IntakeOut() {
    intake.set(-0.2);
  }

  public void boomWenchOff() {
    intake.stopMotor();
  }
  
}
