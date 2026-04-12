package frc.robot.submodule.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// 8ft and 2.44 meters with the power given to the motors

public class Intake extends SubsystemBase{

    // Referencing motor controller object-
    // orange motors
    // private final SparkMax ShooterMotor = new SparkMax(16, MotorType.kBrushless);
    // private final SparkMax Indexer = new SparkMax(4, MotorType.kBrushless);
    // private final RelativeEncoder Shooter_encoder = ShooterMotor.getEncoder();
    // private final RelativeEncoder Indexer_Encoder = Indexer.getEncoder();

    // black motors
    private final SparkMax ShooterMotor = new SparkMax(1, MotorType.kBrushless);
    private final RelativeEncoder Shooter_encoder = ShooterMotor.getEncoder();
    private final TalonSRX Indexer = new TalonSRX(12);
    private final TalonSRX MoveIntake = new TalonSRX(11);
    private final SparkMax Intake = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax FeedShooter = new SparkMax(2, MotorType.kBrushless);
    private final RelativeEncoder Intake_Encoder = Intake.getEncoder();    

    private final DigitalInput Out_Switch = new DigitalInput(0);
    private final DigitalInput In_Swich = new DigitalInput(8); 
    public enum MotorState{
        forward,
        backward,
        off,
        FULLSPEEDAHEAD
    }

    public Intake(){

        MoveIntake.setNeutralMode(NeutralMode.Brake);

    }


    // public void TempIntake_setSpeed(double speed){
    //     TempIntake.set(TalonSRXControlMode.PercentOutput, speed);
    // }

    // public double getTempShooter_speed(){
    //     return TempEncoder.getVelocity();
    // }

    // public void TempShooter_setVolts(Voltage volts){
    //     TempShooter.setVoltage(volts.times(1));
    // }

    public Command TESTMOTOR(){
        return runEnd(()->setIndexer_motorSpeed(MotorState.forward), ()->setIndexer_motorSpeed(MotorState.off));
    }
    public Command IntakeOnly(){
        return runEnd(()->setIntake_motorSpeed(MotorState.forward),()->setIntake_motorSpeed(MotorState.off));
    }
    public Command intakeOut(){
        return runEnd(()-> setIntake_motorSpeed(MotorState.backward), ()->setIntake_motorSpeed(MotorState.off));
    }

    public Command FULL_INTAKE(){
        return runEnd(()->setIntake_motorSpeed(MotorState.FULLSPEEDAHEAD), ()->setIntake_motorSpeed(MotorState.off));
    }
    //keep this or else break!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public void DisableSolenoids(){
    }

    //This command will run the intake of the robot. Then set to 0 speed when false.
    public Command IntakePrep(){
            return runEnd(()->MoveIntake.set(TalonSRXControlMode.PercentOutput, 0.25),()-> MoveIntake.set(TalonSRXControlMode.PercentOutput, 0)).until(() -> Out_Switch.get());
    }
    
    public void setMoveIndex_Speed(boolean Forward){
        // if(Forward == true){
        //     while(In_Swich.get()){
        //     MoveIntake.set(TalonSRXControlMode.PercentOutput,0.25);
        //     }
        // }
        // else{
        //     while (Out_Switch.get()){

        //      MoveIntake.set(TalonSRXControlMode.PercentOutput,-0.25);    
        //     }
                
        // }
        // MoveIntake.set(TalonSRXControlMode.PercentOutput, 0);
    }

  public boolean Exstended_SolenoidState(){

        return true;

    }


 @Override
    public void periodic() {
        SmartDashboard.putNumber("Current RPM", getShooter_motorSpeed());
        SmartDashboard.putNumber("Current Intake RPM", getIntake_Speed());
        // SmartDashboard.putNumber("TempRPM", getTempShooter_speed());
        SmartDashboard.putNumber("AMP Out", MoveIntake.getStatorCurrent());
        SmartDashboard.putNumber("SUPPLY CURRENT", MoveIntake.getSupplyCurrent());
        SmartDashboard.putBoolean("IN_Swich", In_Swich.get());
        SmartDashboard.putBoolean("Out_Swich", Out_Switch.get());
    }
    //methods that will run the motors.
    //Curerently shooter can shoot successfully from 8ft and 2.44 meters.
  
public Command OutAndDrop(){
    return Commands.runEnd(() -> setMoveIndex_motorSpeed(MotorState.forward), () -> setMoveIndex_motorSpeed(MotorState.off)).until(() -> Out_Switch.get());
}
public Command UpAndIn(){
    return Commands.runEnd(() -> setMoveIndex_motorSpeed(MotorState.backward), () -> setMoveIndex_motorSpeed(MotorState.off)).until(() -> In_Swich.get());
}

    public double getShooter_motorSpeed(){
        return Shooter_encoder.getVelocity();
    }

    public double getIntake_motorSpeed(){
        return Intake_Encoder.getVelocity();
    }

    public void setShooter_motorSpeed(MotorState state){
        switch (state){
        case off:
        ShooterMotor.set(0);
        break;
        case forward:
        ShooterMotor.set(-0.8);
        break;
        case backward:
        ShooterMotor.set(0.8);
        break;
        }
        
    }
    public void setFeedShooter_motorSpeed(MotorState state){
        switch (state) {
            case forward:
                 FeedShooter.set(-0.25);
                break;
            case off:
                 FeedShooter.set(0);
                break;
        }
        
    }

    public void setIndexer_motorSpeed(MotorState state){
        switch (state) {
            case off:
                Indexer.set(TalonSRXControlMode.PercentOutput,0);
                break;
            case forward:
                Indexer.set(TalonSRXControlMode.PercentOutput,0.60);
                break;
            case backward:
                Indexer.set(TalonSRXControlMode.PercentOutput,-0.60);
                break;
        }
    }

    public void setMoveIndex_motorSpeed(MotorState state) {
        switch (state) {
            case off:
                MoveIntake.set(TalonSRXControlMode.PercentOutput, 0);
                break;
            case forward:
                MoveIntake.set(TalonSRXControlMode.PercentOutput, 0.5);
                break;
            case backward:
                MoveIntake.set(TalonSRXControlMode.PercentOutput, -0.5);
                break;
            default:
                MoveIntake.set(TalonSRXControlMode.PercentOutput, 0);
                break;
        }
    }



    public void setShooter_motorVoltage(Voltage voltage){
        ShooterMotor.setVoltage(voltage.times(1));
    }

    public void setIntake_motorSpeed(MotorState state){
        switch (state){
        case off:
        Intake.set(0);
        break;
        case forward:
        Intake.set(-0.65);
        break;
        case backward:
        Intake.set(0.65);
        break;
        case FULLSPEEDAHEAD:
        Intake.set(1);
        break;
        }
    }
    public void SetIntake_Voltage(double volts){
        Intake.setVoltage(volts);
    }

    public double getIntake_Speed(){
     return Intake_Encoder.getVelocity();   
    }


      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

      // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              this::setShooter_motorVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(ShooterMotor.getAppliedOutput() * ShooterMotor.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(Shooter_encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(Shooter_encoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}