package frc.robot.submodule.Commands;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class ShooterMech extends Command{

private final Intake subSystem;

public ShooterMech(Intake shooterSubsystem){
    subSystem = shooterSubsystem;

}

@Override
public void execute(){
   if(subSystem.Exstended_SolenoidState()){
   
    subSystem.Shooter_motorVoltage(Voltage.ofBaseUnits(7,Volt));

    if(Math.abs(subSystem.getShooter_motorSpeed()) > 3350){
        
        subSystem.Indexer_motorSpeed(0.3);
        subSystem.Intake_motorSpeed(-0.3);
        
        

    }
    }

    else{

    subSystem.Shooter_motorSpeed(0);
    subSystem.Indexer_motorSpeed(0);
    subSystem.Intake_motorSpeed(0);

    }



}
@Override
public void end(boolean interrupted){
    subSystem.Shooter_motorSpeed(0);
    subSystem.Indexer_motorSpeed(0);
    subSystem.Intake_motorSpeed(0);
}


    
}
