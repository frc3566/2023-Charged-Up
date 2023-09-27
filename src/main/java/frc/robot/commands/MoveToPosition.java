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
        System.out.println("Waiting for trajectory...");
        
        vision.getTrajectory().ifPresentOrElse(trajectory -> {
            System.out.println("Running vision trajectory");

            var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

            SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
            
            s_Swerve.resetOdometry(trajectory.getInitialPose());
            swerveControllerCommand.schedule();
            cancelCommand = false;
        }, () -> {
            cancelCommand = true;
            DriverStation.reportWarning("Unable to generate vision trajectory", false);
        });
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
