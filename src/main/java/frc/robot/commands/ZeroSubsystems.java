package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class ZeroSubsystems extends CommandBase{
    Swerve swerve;
    Elevator elevator;

    public ZeroSubsystems(Swerve swerve, Elevator elevator) {
        this.swerve = swerve;
        this.elevator = elevator;

        addRequirements(swerve, elevator);
    }

    @Override
    public void execute() {
        elevator.setZero();
        swerve.zeroGyro();
    }
}
// 