package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class ZeroSubsystems extends CommandBase{
    Swerve swerve;
    Arm arm;
    Elevator elevator;
    int i;

    public ZeroSubsystems(Swerve swerve, Arm arm, Elevator elevator) {
        this.swerve = swerve;
        this.arm = arm;
        this.elevator = elevator;

        addRequirements(swerve, arm, elevator);
    }

    @Override

    public void initialize() {
       swerve.zeroGyro();
    }

    public void execute() {}

    public void end(boolean interrupted) {}
    public boolean isFinished() {
      return true;
    }
}
// 