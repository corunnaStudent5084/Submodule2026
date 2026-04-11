package frc.robot.submodule.subCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.submodule.subsystems.Intake;
import frc.robot.submodule.subsystems.Intake.MotorState;



public class RunIntake extends Command {

    private final Intake intakesub;
//runIntake constructor

    public RunIntake(Intake intakeSubsystem){
    
     intakesub = intakeSubsystem; 
    }
    
    @Override
    public void execute() {
        // intakesub.setShooter_motorSpeed(0.875);
            // intakesub.setIndexer_motorSpeed(MotorState.forward);
            intakesub.setIntake_motorSpeed(MotorState.forward);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // intakesub.setShooter_motorSpeed(0);
        // intakesub.setIndexer_motorSpeed(MotorState.off);
        intakesub.setIntake_motorSpeed(MotorState.off);
    }


}
