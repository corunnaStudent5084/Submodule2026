package frc.robot.submodule.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

// DO NOT MOVE WHAT IS ABOVE!! >:|

public class ExstendsIntake extends Command {

    Intake subsystem;

    public ExstendsIntake(Intake subsystem){

        this.subsystem = subsystem;

    }

    @Override
    public void execute() {
        subsystem.MoveExstendIntakeSolenoid(true);
    }

    @Override
    public void end(boolean interrupted) {
        if(subsystem.DropIntake_SolenoidState() == false){
            subsystem.MoveExstendIntakeSolenoid(false);
        }
     
    }


    
}