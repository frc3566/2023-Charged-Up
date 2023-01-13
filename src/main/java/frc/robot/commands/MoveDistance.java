package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class MoveDistance {
    private DrivetrainSubsystem drive;
    private PIDController controller = new PIDController(Constants.DRIVETRAIN_DISTANCE_GAINS.kP, Constants.DRIVETRAIN_DISTANCE_GAINS.kI, Constants.DRIVETRAIN_DISTANCE_GAINS.kD);

    double distance;

    public MoveDistance(double distance, DrivetrainSubsystem drive) {
      this.distance = distance;
      this.drive = drive;
    }

    public void initialize() {
      controller.reset();
      
    }
}
