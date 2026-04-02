package frc.robot.submodule.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.submodule.subsystems.Intake;

// DO NOT MOVE WHAT IS ABOVE!! >:|

public class ExstendsIntake extends Command {

    Intake subsystem;

    public ExstendsIntake(Intake subsystem){

        this.subsystem = subsystem;

    }

    @Override
    public void execute() {
        subsystem.MoveExstendIntakeSolenoid(true);
        subsystem.OutAndDrop();
    }

    @Override
    public void end(boolean interrupted) {
            subsystem.UpAndIn();
            subsystem.MoveExstendIntakeSolenoid(false);
     
    }


    
}