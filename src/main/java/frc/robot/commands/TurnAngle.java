package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class TurnAngle extends CommandBase {
    private Swerve swerve;
    private double angle;
    private PIDController controller = new PIDController(
        Constants.Swerve.angleKP, 
        Constants.Swerve.angleKI, 
        Constants.Swerve.angleKD);

    public TurnAngle(Swerve swerve, double angle) {
        this.swerve = swerve;
        this.angle = angle;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        controller.reset();
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
