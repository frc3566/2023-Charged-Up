package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakePosition extends CommandBase {    
    private Arm arm;
    private Elevator elevator;
    private double extensionMax;

    public IntakePosition(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;

        addRequirements(arm, elevator);

        extensionMax = 37.5;
    }

    @Override
    public void execute() {

        double angle = Math.toRadians(80 - arm.getCANCoderPosition());
        double extension = elevator.getLength() * Math.cos(angle);
        double correction = (extensionMax/Math.cos(angle) - 5) / 50;
        if(extension > extensionMax){
            if(arm.getPower() != 0 ){
                elevator.setExtension(correction);
            }
            // System.out.println( "Elevator Correction: " + correction);
            if(elevator.getPower() > 0){
                elevator.off();
            }
        }
        // System.out.println("Elevator Length: " + elevator.getLength());
        // System.out.println("Frame Extension: " + extension);
        // System.out.println("Theta: " + angle);

    }
}