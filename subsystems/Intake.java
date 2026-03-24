package frc.robot.submodule.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.ExstendsIntake;

// 8ft and 2.44 meters with the power given to the motors

public class Intake extends SubsystemBase{

    // Referencing motor controller object-
    // orange motors
    // private final SparkMax ShooterMotor = new SparkMax(16, MotorType.kBrushless);
    // private final SparkMax Indexer = new SparkMax(4, MotorType.kBrushless);
    // private final RelativeEncoder Shooter_encoder = ShooterMotor.getEncoder();
    // private final RelativeEncoder Indexer_Encoder = Indexer.getEncoder();

    // black motors
    private final SparkMax ShooterMotor = new SparkMax(17, MotorType.kBrushless);
    private final RelativeEncoder Shooter_encoder = ShooterMotor.getEncoder();
    private final SparkMax Indexer = new SparkMax(15,MotorType.kBrushless);

    // maybe shared?
    private final SparkMax Intake = new SparkMax(14, MotorType.kBrushless);
        private final RelativeEncoder Intake_Encoder = Intake.getEncoder();
    private final Solenoid ExstendIntake_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid DropIntake_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    private final Compressor Compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
    // public void TempIntake_setSpeed(double speed){
    //     TempIntake.set(TalonSRXControlMode.PercentOutput, speed);
    // }

    // public double getTempShooter_speed(){
    //     return TempEncoder.getVelocity();
    // }

    // public void TempShooter_setVolts(Voltage volts){
    //     TempShooter.setVoltage(volts.times(1));
    // }



    //This command will run the intake of the robot. Then set to 0 speed when false.
    
    public Command Intakeforward(){

        return runOnce(() ->MoveExstendIntakeSolenoid(true));

    } 

    public Command DropIntake(){

        return runOnce(() ->DropIntake_Solenoid.set(true)).onlyIf(() ->Exstended_SolenoidState());

    }

    public Command PickUpIntake(){

        return runOnce(() ->DropIntake_Solenoid.set(false));
    }

    public Command OutAndDrop(){
        return runOnce(() ->ExstendIntake_solenoid.set(true)).andThen(Commands.waitSeconds(2)).andThen(DropIntake());
    }

    public Command UpAndIn(){
        return PickUpIntake().andThen(Commands.waitSeconds (1)).andThen(runOnce(() ->ExstendIntake_solenoid.set(false)).unless(() ->DropIntake_SolenoidState()));
    }

    public void DisableSolenoids(){
        ExstendIntake_solenoid.set(false);
        DropIntake_Solenoid.set(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current RPM", getShooter_motorSpeed());
        // SmartDashboard.putNumber("TempRPM", getTempShooter_speed());
    }
    


    //methods that will run the motors.
    //Curerently shooter can shoot successfully from 8ft and 2.44 meters.
    public boolean Exstended_SolenoidState(){

        return ExstendIntake_solenoid.get();

    }

    public boolean DropIntake_SolenoidState(){
        return DropIntake_Solenoid.get();
    }

    public double getShooter_motorSpeed(){
        return Shooter_encoder.getVelocity();
    }

    public double getIntake_motorSpeed(){
        return Intake_Encoder.getVelocity();
    }

    public void Shooter_motorSpeed(double speed){
        ShooterMotor.set(-speed);
    }

    public void Indexer_motorSpeed(double speed){
        Indexer.set(speed);
    }

    public void MoveExstendIntakeSolenoid(boolean on){
        
        ExstendIntake_solenoid.set(on);
    }

     public void MoveDropIntakeSolenoid(boolean on){

        DropIntake_Solenoid.set(on);
    }

    public void Shooter_motorVoltage(Voltage voltage){
        ShooterMotor.setVoltage(voltage.times(1));
    }

    public void Intake_motorSpeed(double speed){
        Intake.set(speed);
    }

    public double Intake_getSpeed(){
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
              this::Shooter_motorVoltage,
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