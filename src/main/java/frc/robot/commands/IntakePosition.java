package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakePosition extends CommandBase {    
    private Arm arm;
    private Elevator elevator;

    public IntakePosition(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        arm.setAngle(0);
        elevator.setExtendtionPercentage(0.8);
    }
}