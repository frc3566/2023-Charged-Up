package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeControl extends CommandBase{
    DoubleSupplier LT;
    DoubleSupplier RT;
    Intake intake;

    public IntakeControl(Intake intake, DoubleSupplier LT, DoubleSupplier RT) {
        this.LT = LT;
        this.RT = RT;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPower(0);
        if(this.LT.getAsDouble() > 0.5){
            intake.setPower(-0.2);
        }
        if(this.RT.getAsDouble() > 0.5){
            intake.setPower(0.2);
        }
    }
}
