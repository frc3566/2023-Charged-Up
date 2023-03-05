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
        double lt = LT.getAsDouble(), rt = RT.getAsDouble();
        if (lt == 0 && rt == 0 || lt > 0 && rt > 0) {
            intake.off();
            return;
        }
        double power = Math.max(lt, rt) * ( lt > rt ? -1 : 1) * -0.5;
        // double power = 0.5;
        intake.setPower(power);
    }
}
// 