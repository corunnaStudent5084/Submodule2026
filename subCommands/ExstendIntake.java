package frc.robot.submodule.subCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.submodule.subsystems.Intake;

public class ExstendIntake extends Command{
     private final Intake subsystem; 
     public ExstendIntake(Intake subsystem){ 
    this.subsystem = subsystem; } 


@Override public void execute() {
   subsystem.setMoveIndex_Speed(true);
}

@Override
public void end(boolean interrupted) {
     subsystem.setMoveIndex_Speed(false);
}



}