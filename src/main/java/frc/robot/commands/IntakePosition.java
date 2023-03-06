package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakePosition extends CommandBase {    
    private Arm arm;
    private Elevator elevator;
    private double extensionMax = 37.5;
    private double extensionHeightMax = 78 - 35; // Height limit - pivot hieght

    public IntakePosition(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;

        addRequirements(arm, elevator);

        extensionMax = 37.5;
    }

    @Override
    public void execute() {

        double angle = Math.toRadians(90 - arm.getCANCoderPosition());

        //horizontal
        double extension = elevator.getLength() * Math.cos(angle);
        double correction = (extensionMax/Math.cos(angle) - 5) / 50;

        //vertical
        double heightExtension = elevator.getLength() * Math.sin(angle);
        double heightCorrection = (extensionHeightMax/Math.sin(angle) - 5) / 50;
        // System.out.println(heightCorrection);
        correction = Math.min(correction, heightCorrection);
        if(extension > extensionMax || heightExtension > extensionHeightMax){
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