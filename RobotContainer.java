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
import java.util.function.DoubleSupplier;

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
    private final Joystick driver1 = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    public static double speedCoefficient = 1.0;

    /* Driver Buttons */
    private final JoystickButton X = new JoystickButton(driver1, XboxController.Button.kX.value);
    private final JoystickButton Y = new JoystickButton(driver1, XboxController.Button.kY.value);
    private final JoystickButton A = new JoystickButton(driver1, XboxController.Button.kA.value);
    private final JoystickButton B = new JoystickButton(driver2, XboxController.Button.kB.value);
    private final JoystickButton RB = new JoystickButton(driver1, XboxController.Button.kRightBumper.value);
    private final JoystickButton LB = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
    private final int LTAxis = XboxController.Axis.kLeftTrigger.value;
    private final int RTAxis = XboxController.Axis.kRightTrigger.value;
    private final POVButton DPadUp = new POVButton(driver2, 0);
    private final POVButton DPadDown = new POVButton(driver2, 180);
    private final POVButton DPadLeft = new POVButton(driver2, 90);
    private final POVButton DPadRight = new POVButton(driver2, 270);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private Vision vision;


    /* The container for the robot. Contains subsystems, OI devices, and commands. 
     * @throws IOException*/
    public RobotContainer() throws IOException {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver1.getRawAxis(translationAxis), 
                () -> -driver1.getRawAxis(strafeAxis), 
                () -> -driver1.getRawAxis(rotationAxis), 
                () -> A.getAsBoolean()
            )
        );

        intake.setDefaultCommand(new IntakeControl(intake, () -> driver2.getRawAxis(LTAxis), () -> driver2.getRawAxis(RTAxis)));

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
        X.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        DPadUp.onTrue(new InstantCommand(() -> arm.setPower(0.5)));
        DPadUp.onFalse(new InstantCommand(() -> arm.off()));

        DPadDown.onTrue(new InstantCommand(() -> arm.setPower(-0.5)));
        DPadDown.onFalse(new InstantCommand(() -> arm.off()));

        DPadLeft.onTrue(new InstantCommand(() -> elevator.setPower(0.5)));
        DPadLeft.onFalse(new InstantCommand(() -> elevator.off()));

        DPadRight.onTrue(new InstantCommand(() -> elevator.setPower(-0.5)));
        DPadRight.onFalse(new InstantCommand(() -> elevator.off()));

        RB.onTrue(new InstantCommand(() -> s_Swerve.increaseSpeed()));
        LB.onTrue(new InstantCommand(() -> s_Swerve.decreaseSpeed()));
        Y.onTrue(new MoveToPosition(s_Swerve, vision));

        B.onTrue(new IntakePosition(arm, elevator));
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
