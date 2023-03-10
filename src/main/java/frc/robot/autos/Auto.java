package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto extends SequentialCommandGroup{
    public static final double coefficient = 1.2;
    

    public Auto(Swerve s_Swerve, Elevator elevator, Arm arm, Intake intake) {
        TrajectoryConfig config1 =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        TrajectoryConfig config2 =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);
        config2.setReversed(true);

        // An example trajectory to follow.  All units in meters.
        Trajectory traj1 = //TODO: Fix auto code
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1.0 * coefficient, 0 * coefficient), new Translation2d(4.0 * coefficient, 0 * coefficient)),
                // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(1.5 * coefficient, 0 * coefficient, Rotation2d.fromDegrees(90)), // example
                new Pose2d(5 * coefficient, 0 * coefficient, Rotation2d.fromDegrees(0)), 

                config1);
        Trajectory traj2 = 
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0.5 * coefficient, 0.0 * coefficient), new Translation2d(2.5 * coefficient, 0.0 * coefficient)),
                // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(1.5 * coefficient, 0 * coefficient, Rotation2d.fromDegrees(90)), // example
                new Pose2d(3 * coefficient, 0 * coefficient, Rotation2d.fromDegrees(0)), 

                config2);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                traj1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    
            SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    traj2,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(traj1.getInitialPose())),
            // new InstantCommand(() -> intake.setPower(0.5)),
            // // new WaitCommand(2), 
            // new InstantCommand(() -> arm.setAngle(30)),
            // // new WaitCommand(2),
            // new InstantCommand(() -> elevator.setExtension(1)), 
            // new InstantCommand(() -> intake.setPower(-0.75)), 
            // // new WaitCommand(2), 
            // new InstantCommand(() -> intake.off())
            swerveControllerCommand1,
            
            new InstantCommand(() -> s_Swerve.resetOdometry(traj2.getInitialPose())),
            swerveControllerCommand2
        );
    }
}
