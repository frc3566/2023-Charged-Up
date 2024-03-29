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
    private boolean cancelCommand;
    private boolean auto;
    private double balanceThreshold = 2;
    int dir;

    public AutoBalancing(Swerve swerve, boolean auto) {
        this.swerve = swerve;
        this.auto = auto;
        addRequirements(swerve);

        linearController = new PIDController(0.05, 0.0, 0.5); //TODO TUNE THIS SHIT
        linearController.setTolerance(1);
        linearController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        cancelCommand = false;
        return;
    }

    public void execute() {
        double pos = swerve.getRoll().getDegrees();
        // System.out.println("PITCH" + swerve.getPitch().getDegrees());
        // System.out.println("Activated" + pos);
        PIDVal = linearController.calculate(pos, 0);
        if (pos < 0)
            dir = 1;
        if (pos > 0)
            dir = -1;
        if (MathUtil.clip(PIDVal, -0.4, 0.4) < 0.05 && auto == true) {
            cancelCommand = true;
        }

        System.out.println(Math.abs(MathUtil.clip(PIDVal, -0.4, 0.4)) * dir * Constants.Swerve.maxSpeed);
        this.swerve.drive(
            new Translation2d(Math.abs(MathUtil.clip(PIDVal, -0.4, 0.4)) * dir * Constants.Swerve.maxSpeed, 0), 
            0, 
            false, 
            true
        );

        Timer.delay(0.005);	
        }

    public void end(boolean interrupted) {
        cancelCommand = true;
    }
    public boolean isFinished() {
      return cancelCommand;
    }
}