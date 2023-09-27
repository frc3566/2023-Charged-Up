package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.List;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class MoveToPosition extends CommandBase {
    /** Moves the robot to the desired position */

    // Main defines;
    public static final double coefficient = 1.2;
    private boolean cancelCommand;
    private SwerveControllerCommand swerveControllerCommand;
    Swerve s_Swerve;
    Vision vision;

    public MoveToPosition(Swerve s_Swerve, Vision vision) {
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve, vision);
    }

    @Override
    public void initialize() {
        System.out.println("Waiting for trajectory");
        
        var toTargetTrajectory = vision.getTrajectory();
        if (toTargetTrajectory.isEmpty()) {
            DriverStation.reportWarning("Unable to generate trajectory", false);
            // cancelCommand = true;
            // return;
        }

        System.out.println("Has trajectory");

        // Trajectory teleopTrajectory = toTargetTrajectory.get();

        //TODO: Delete following segment
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory teleopTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0.0, 0.0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1.0 * coefficient, 0 * coefficient), new Translation2d(1.0 * coefficient, 0 * coefficient)),
                new Pose2d(2.0 * coefficient, 0 * coefficient, Rotation2d.fromDegrees(0)), 
                config);


        System.out.println("Running Teleop Trajectory.");

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        swerveControllerCommand =
            new SwerveControllerCommand(
                teleopTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        // s_Swerve.resetOdometry(vision.getTrajectory().get().getInitialPose());
        s_Swerve.resetOdometry(teleopTrajectory.getInitialPose());
        swerveControllerCommand.schedule();
        cancelCommand = false;
        return;
    }
    
    @Override
    public void execute() {
        if (swerveControllerCommand.isFinished())
            cancelCommand = true;
    }

    public void end(boolean interrupted) {
        if (swerveControllerCommand.isFinished() == false)
            swerveControllerCommand.cancel();
    }
    public boolean isFinished() {
        return cancelCommand;
    }
}
