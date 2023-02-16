package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PickUp extends CommandBase{
    Elevator elevator;
    Intake intake;
    public PickUp(Elevator elevator, Intake intake){
        this.elevator = elevator;
        this.intake = intake;
        addRequirements(elevator, intake);
    }
    public void execute(){
        elevator.fullyDown();
        intake.closeBoth();
        elevator.fullyUp();
        intake.open();
    }
}
