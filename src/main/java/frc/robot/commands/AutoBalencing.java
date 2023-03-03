package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalencing extends CommandBase {    
    private Swerve swerve;
    private PIDController linearController;
    private double PIDVal;

    public AutoBalencing(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        linearController = new PIDController(0.2, 0.0, 0);
        linearController.setTolerance(0.5);
        linearController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        swerve.off();
        return;
    }

    public void execute() {
        double PIDVal = this.linearController.calculate(0, swerve.getPitch().getRadians());
        this.swerve.drive(
            new Translation2d(0, MathUtil.clip(PIDVal, -0.5, 0.5) * Constants.Swerve.maxSpeed), 
            0, 
            true, 
            true
        );
    }

    public void end(boolean interrupted) {}
    public boolean isFinished() {
      return PIDVal == 0;
    }
}