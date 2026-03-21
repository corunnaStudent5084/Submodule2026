package frc.robot.submodule.Commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ShootWithVelocityControl extends Command {
    private Intake intake;
    private double currentRPM = 0;

    private SimpleMotorFeedforward feedforwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    private double feedForwardValue = 0;

    private PIDController pidCalculator = new PIDController(0, 0, 0);
    private double pidValue = 0;


    private double targetRPM = 0;//3000
    private double ks = 0;//0.15
    private double kv = 0;//0.002007
    private double ka = 0;//0
    private double p = 0;//0.005
    private double i = 0;//0
    private double d = 0;//0.0007
    private double marginOfError = 100;

/* Ks 0.125 just a hair higher stops at some points
 * 
 * 
 * 
*/


    public ShootWithVelocityControl(Intake intakeSystem) {
        intake = intakeSystem;
        addRequirements(intake);
        Preferences.initDouble("Target RPM", targetRPM);
        Preferences.initDouble("ks", ks);
        Preferences.initDouble("kv", kv);
        Preferences.initDouble("ka", ka);
        Preferences.initDouble("p", p);
        Preferences.initDouble("i", i);
        Preferences.initDouble("d", d);
        Preferences.initDouble("Margin of Error", marginOfError);
    }

    @Override
    public void initialize() {
        targetRPM = Preferences.getDouble("Target RPM", targetRPM);
        ks = Preferences.getDouble("ks", ks);
        kv = Preferences.getDouble("kv", kv);
        ka = Preferences.getDouble("ka", ka);
        p = Preferences.getDouble("p", p);
        i = Preferences.getDouble("i", i);
        d = Preferences.getDouble("d", d);
        marginOfError = Preferences.getDouble("Margin of Error", marginOfError);

        feedforwardCalculator.setKs(ks);
        feedforwardCalculator.setKv(kv);
        feedforwardCalculator.setKa(ka);

        pidCalculator.setPID(p, i, d);
    }

    @Override
    public void execute() {
        currentRPM = intake.getShooter_motorSpeed();
        feedForwardValue = feedforwardCalculator.calculate(targetRPM);
        pidValue = pidCalculator.calculate(currentRPM, targetRPM);

        intake.Shooter_motorVoltage(Volts.of(feedForwardValue + pidValue));

        if(Math.abs(intake.getShooter_motorSpeed() - targetRPM) <= marginOfError && targetRPM != 0){
            intake.Indexer_motorSpeed(0.85);
        }
    
        SmartDashboard.putBoolean("okToShoot", Math.abs(intake.getShooter_motorSpeed() - targetRPM) <= marginOfError && targetRPM != 0);
        SmartDashboard.putNumber("ShooterSpeed", targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        intake.Shooter_motorVoltage(Volts.zero());
        intake.Indexer_motorSpeed(0);
    }
    
}
