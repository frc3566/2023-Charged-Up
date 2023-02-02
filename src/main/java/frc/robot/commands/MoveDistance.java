package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class MoveDistance extends CommandBase {
    private Swerve swerve;
    private double distance, angle;

    private PIDController controller = new PIDController(
        Constants.Swerve.driveKP, 
        Constants.Swerve.driveKI, 
        Constants.Swerve.driveKD);

    public MoveDistance(Swerve swerve, int distance, int angle) {
        this.swerve = swerve;
        this.distance = distance;
        this.angle = angle;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        controller.reset();
        distance += getAveragePosition();
        swerve.drive(new Translation2d(0, 0), angle, false, false);
    }
    
    public double getAveragePosition() {
        SwerveModulePosition[] positions = swerve.getModulePositions();
        double average = 0.0;
        for (SwerveModulePosition mod : positions) {
            average += mod.distanceMeters;
        }
        average /= positions.length;
        return average;
    }

    @Override
    public void execute() {
        double linearPID = controller.calculate(getAveragePosition(), distance);
        double velocity = Constants.Swerve.driveKS * Math.signum(linearPID) + linearPID;
        swerve.drive(new Translation2d(velocity, velocity), 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(distance - getAveragePosition()) <= 0.05;
    }
}
