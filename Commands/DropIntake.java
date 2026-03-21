package frc.robot.submodule.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DropIntake extends Command {

    Intake subsystem;

    public DropIntake(Intake subsystem){

        this.subsystem = subsystem;

    }

    @Override
    public void execute() {
        
        subsystem.MoveDropIntakeSolenoid(true);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.MoveDropIntakeSolenoid(false);
    }


}
