package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    public static double speedCoefficient = 1.0;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);

    private final POVButton boomWenchUp = new POVButton(driver, 0);
    private final POVButton boomWenchDown = new POVButton(driver, 180);
    private final POVButton telescopingWenchIn = new POVButton(driver, 90);
    private final POVButton telescopingWenchOut = new POVButton(driver, 270);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton increaseSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton decreaseSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton runTrajectory = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private Vision vision;


    /* The container for the robot. Contains subsystems, OI devices, and commands. 
     * @throws IOException*/
    public RobotContainer() throws IOException {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        vision = new Vision();
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        boomWenchUp.onTrue(new InstantCommand(() -> arm.boomWenchUp()));
        boomWenchUp.onFalse(new InstantCommand(() -> arm.boomWenchOff()));

        boomWenchDown.onTrue(new InstantCommand(() -> arm.boomWenchDown()));
        boomWenchDown.onFalse(new InstantCommand(() -> arm.boomWenchOff()));

        telescopingWenchOut.onTrue(new InstantCommand(() -> arm.telescopingWenchOut()));
        telescopingWenchOut.onFalse(new InstantCommand(() -> arm.telescopingWenchOff()));

        telescopingWenchIn.onTrue(new InstantCommand(() -> arm.telescopingWenchIn()));
        telescopingWenchIn.onFalse(new InstantCommand(() -> arm.telescopingWenchOff()));

        increaseSpeed.onTrue(new InstantCommand(() -> s_Swerve.increaseSpeed()));
        decreaseSpeed.onTrue(new InstantCommand(() -> s_Swerve.decreaseSpeed()));
        runTrajectory.onTrue(new MoveToPosition(s_Swerve, vision));
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
