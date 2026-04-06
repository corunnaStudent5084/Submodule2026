package frc.robot.submodule.subCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.submodule.subsystems.Intake;

public class ExstendIntake extends Command{
     private final Intake subsystem; 
     public ExstendIntake(Intake subsystem){ 
    this.subsystem = subsystem; } 


@Override public void execute() {
    execute();
}

@Override
public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
}



}