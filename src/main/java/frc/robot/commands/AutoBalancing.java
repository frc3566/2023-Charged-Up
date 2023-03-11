package frc.robot.commands; //TODO on off switch

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalancing extends CommandBase {    
    private Swerve swerve;
    private PIDController linearController;
    private double PIDVal;
    private boolean cancelCommand = false;
    private boolean on;
    private double balanceThreshold = 2;
    int dir;

    public AutoBalancing(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        linearController = new PIDController(0.1, 0.0, 0);
        linearController.setTolerance(1);
        linearController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        return;
    }

    public void execute() {
        double pos = swerve.getRoll().getDegrees();
        System.out.println("Activated" + pos);
        PIDVal = linearController.calculate(pos, 0);
        if (pos < 0)
            dir = 1;
        if (pos > 0)
            dir = -1;
        if (MathUtil.clip(PIDVal, -0.4, 0.4) < 0.05) {
            cancelCommand = true;
        }

        System.out.println("Speed: " + MathUtil.clip(PIDVal, -0.4, 0.4));
        this.swerve.drive(
            new Translation2d(Math.abs(MathUtil.clip(PIDVal, -0.4, 0.4)) * dir, 0), 
            0, 
            false, 
            true
        );

        Timer.delay(0.005);	
        }

    public void off() {
        on = false;
    }

    public void on() {
        on = true;
    }

    public void end(boolean interrupted) {
        cancelCommand = true;
    }
    public boolean isFinished() {
      return cancelCommand;
    }
}