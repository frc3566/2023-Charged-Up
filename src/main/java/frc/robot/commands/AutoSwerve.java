package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private Translation2d translation;
    private double rotation;
    private int count;

    public AutoSwerve(Swerve s_Swerve, Translation2d translation, double rotation) {
        this.s_Swerve = s_Swerve;
        this.translation = translation;
        this.rotation = rotation;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        s_Swerve.drive(translation, rotation, true, true);
    }
    public boolean isFinished() {
        return count == 0;
    }
}