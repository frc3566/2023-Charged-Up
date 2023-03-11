package frc.robot.autos;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.util.Units;
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

public class HardCodeAuto extends SequentialCommandGroup{
    public static final double coefficient = 1.2;
    

    public HardCodeAuto(Swerve s_Swerve, Elevator elevator, Arm arm, Intake intake) {


        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new AutoSwerve(s_Swerve, new Translation2d(5, 0), 0),
            new WaitCommand(2), 
            new AutoSwerve(s_Swerve, new Translation2d(-2, 0), Units.degreesToRadians(180.0))

            // new InstantCommand(() -> intake.setPower(0.5)),
            // // new WaitCommand(2), 
            // new InstantCommand(() -> arm.setAngle(30)),
            // // new WaitCommand(2),
            // new InstantCommand(() -> elevator.setExtension(1)), 
            // new InstantCommand(() -> intake.setPower(-0.75)), 
            // // new WaitCommand(2), 
            // new InstantCommand(() -> intake.off())
        );
    }
}
