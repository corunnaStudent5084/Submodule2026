package frc.robot.submodule.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;



public class RunIntake extends Command {

    private final Intake intakesub;
//runIntake constructor

    public RunIntake(Intake intakeSubsystem){
    
     intakesub = intakeSubsystem; 
    }
    
    @Override
    public void execute() {
        // intakesub.Shooter_motorSpeed(0.875);
            intakesub.Indexer_motorSpeed(-0.8);
            intakesub.Intake_motorSpeed(-0.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // intakesub.Shooter_motorSpeed(0);
        intakesub.Indexer_motorSpeed(0);
        intakesub.Intake_motorSpeed(0);
    }


}
