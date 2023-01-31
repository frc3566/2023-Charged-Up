package frc.robot.commands;

import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveDistance extends CommandBase {
    private Swerve swerve;
    private double distance, angle;

    public MoveDistance(Swerve swerve, int distance, int angle) {
        this.swerve = swerve;
        this.distance = distance;
        this.angle = angle;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
