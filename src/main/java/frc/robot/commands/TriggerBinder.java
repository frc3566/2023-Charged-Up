package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TriggerBinder extends CommandBase {    
    Runnable onTrue;
    Runnable onFalse;
    DoubleSupplier getAxis;
    double gauge;

    public TriggerBinder(Runnable onTrue, DoubleSupplier getAxis, double gauge) {
        this.onTrue = onTrue;
        this.getAxis = getAxis;
        this.gauge = gauge;
    }

    public TriggerBinder(Runnable onTrue,Runnable onFlase, DoubleSupplier getAxis, double gauge) {
        this.onTrue = onTrue;
        this.getAxis = getAxis;
        this.gauge = gauge;
    }

    @Override
    public void execute() {
        if(this.getAxis.getAsDouble() > gauge){
            onTrue.run();
        }
        if(this.getAxis.getAsDouble() <= gauge && onFalse != null){
            onFalse.run();
        }
    }
}
