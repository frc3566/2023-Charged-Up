package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    private final boolean RobotCentric = false;
    private boolean firstRun = true;

    // private final JoystickButton X = new JoystickButton(driver1, XboxController.Button.kX.value);
    // private final JoystickButton Y = new JoystickButton(driver1, XboxController.Button.kY.value);
    // private final JoystickButton A = new JoystickButton(driver1, XboxController.Button.kA.value);
    // private final JoystickButton B = new JoystickButton(driver2, XboxController.Button.kB.value);
    // private final JoystickButton RB = new JoystickButton(driver1, XboxController.Button.kRightBumper.value);
    // private final JoystickButton LB = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton resetPivot = new JoystickButton(driver2, XboxController.Button.kY.value);
    // private final JoystickButton resetElevator = new JoystickButton(driver2, XboxController.Button.kB.value);
    // private final int LTAxis = XboxController.Axis.kLeftTrigger.value;
    // private final int RTAxis = XboxController.Axis.kRightTrigger.value;
    // private final POVButton DPadUp = new POVButton(driver2, 0);
    // private final POVButton DPadDown = new POVButton(driver2, 180);
    // private final POVButton DPadLeft = new POVButton(driver2, 90);
    // private final POVButton DPadRight = new POVButton(driver2, 270);

    // New driver 1 controls
    // Speed modulation buttons
    private final JoystickButton RB = new JoystickButton(driver1, XboxController.Button.kRightBumper.value);
    private final JoystickButton LB = new JoystickButton(driver1, XboxController.Button.kLeftBumper.value);
    // Reset field orientation
    private final JoystickButton X = new JoystickButton(driver1, XboxController.Button.kX.value);
    //Auto Balancing
    private final JoystickButton B = new JoystickButton(driver1, XboxController.Button.kB.value);
    private final JoystickButton B2 = new JoystickButton(driver2, XboxController.Button.kB.value);



    // New driver 2 controls
    // Reset value buttons
    private final JoystickButton resetPivot = new JoystickButton(driver2, XboxController.Button.kRightBumper.value);
    private final JoystickButton resetElevator = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
    // Elevator In and Out
    private final POVButton ElevatorIn = new POVButton(driver2, 90);
    private final POVButton ElevatorOut = new POVButton(driver2, 270);
    // Pivot Up and Down
    private final JoystickButton PivotUp = new JoystickButton(driver2, XboxController.Button.kY.value);
    private final JoystickButton PivotDown = new JoystickButton(driver2, XboxController.Button.kA.value);
    // Intake Controls
    private final int LTAxis = XboxController.Axis.kLeftTrigger.value;
    private final int RTAxis = XboxController.Axis.kRightTrigger.value;
    //Move Arm to 
    private final JoystickButton X2 = new JoystickButton(driver2, XboxController.Button.kX.value);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final ZeroSubsystems zeroSubsystems = new ZeroSubsystems(s_Swerve, arm, elevator);
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
                () -> RobotCentric
            )
        );

        intake.setDefaultCommand(new IntakeControl(intake, () -> driver2.getRawAxis(LTAxis), () -> driver2.getRawAxis(RTAxis)));
        arm.setDefaultCommand(new IntakePosition(arm, elevator));

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
        X.onTrue(new InstantCommand(() -> zeroSubsystems.initialize()));
        X2.onTrue(new InstantCommand(() -> arm.setAngle(30)));

        AutoBalancing autoBalanceCommand;
        // Y.onTrue(new ZeroSubsystems(s_Swerve, arm, elevator, 2));
        B.toggleOnTrue(autoBalanceCommand = new AutoBalancing(s_Swerve, false));
        //B.onFalse(new InstantCommand(() -> autoBalanceCommand.cancel()));

        PivotUp.onTrue(new InstantCommand(() -> arm.setPower(1)));
        PivotUp.onFalse(new InstantCommand(() -> arm.off()));

        PivotDown.onTrue(new InstantCommand(() -> arm.setPower(-1)));
        PivotDown.onFalse(new InstantCommand(() -> arm.off()));

        ElevatorIn.onTrue(new InstantCommand(() -> elevator.setPower(0.5)));
        ElevatorIn.onFalse(new InstantCommand(() -> elevator.off()));

        ElevatorOut.onTrue(new InstantCommand(() -> elevator.setPower(-0.5)));
        ElevatorOut.onFalse(new InstantCommand(() -> elevator.off()));

        RB.onTrue(new InstantCommand(() -> s_Swerve.increaseSpeed()));
        LB.onTrue(new InstantCommand(() -> s_Swerve.decreaseSpeed()));
        // Y.onTrue(new MoveToPosition(s_Swerve, vision));
        // Y.onTrue(new InstantCommand(() -> arm.setAngle(0)));

        // B.onTrue(new IntakePosition(arm, elevator));

        resetPivot.onTrue(new InstantCommand(() -> arm.setZero()));
        resetElevator.onTrue(new InstantCommand(() -> elevator.setZero()));
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new FUCKAuto(s_Swerve, elevator, arm, intake);
    }
}
