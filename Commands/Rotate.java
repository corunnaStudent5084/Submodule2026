package frc.robot.submodule.Commands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.concurrent.locks.Lock;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Rotate extends Command {
    
    private double MaxSpeed;
    private double MaxAngularRate;

    private Translation2d tarPosition2d; 
    
    private final CommandJoystick DriveStick;
    private final CommandSwerveDrivetrain drivetrain;

    private PIDController pidCalculator = new PIDController(0.015, 0, 0);
    // private double p = 0;
    // private double i = 0;
    // private double d = 0;


    

    

    public Rotate(CommandSwerveDrivetrain driveTrain,CommandJoystick joystick, double maxSpeed, double maxAngularRate){
        DriveStick = joystick;
        MaxSpeed = maxSpeed;
        MaxAngularRate = maxAngularRate;
        drivetrain = driveTrain;
        
        pidCalculator.enableContinuousInput(-180,180);
        // Preferences.initDouble("p", p);
        // Preferences.initDouble("i", i);
        // Preferences.initDouble("d", d);
    }

    @Override
    public void initialize() {
        // p = Preferences.getDouble("p", p);
        // i = Preferences.getDouble("i", i);
        // d = Preferences.getDouble("d", d);
        
        //  ForwardController.setPID(p, i, d);
        if((DriverStation.getAlliance().orElse(Alliance.Blue)) == Alliance.Blue){
            tarPosition2d = new Translation2d(4.611, 4.021);
        }
        else {
            
            tarPosition2d = new Translation2d(11.915394, 4.021);

        }

    }

    SwerveRequest.FieldCentric Lock = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
        @Override
        public void execute() {
            // TODO Auto-generated method stub
            super.execute();
            Pose2d CurrentPose = drivetrain.getPose();

            Rotation2d targetAngle = tarPosition2d.minus(CurrentPose.getTranslation()).getAngle();

            targetAngle = targetAngle.plus(Rotation2d.k180deg);

            double distanceFromPose = tarPosition2d.getDistance(CurrentPose.getTranslation());




            drivetrain.setControl(
            Lock.withRotationalRate(pidCalculator.calculate(CurrentPose.getRotation().getDegrees(), targetAngle.getDegrees())*MaxAngularRate) 
            .withVelocityX(-DriveStick.getRawAxis(1)*MaxSpeed)//-DriveStick.getRawAxis(1)
            .withVelocityY(-DriveStick.getRawAxis(0)*MaxSpeed)//-DriveStick.getRawAxis(0)
            // .withVelocityX(DriveStick.getRawAxis(1) * MaxSpeed)       
            //     .withVelocityY(DriveStick.getRawAxis(0) * MaxSpeed)
            );

            

            
            SmartDashboard.putNumber("Distance From Tar in Feet", distanceFromPose*3.28084);
            SmartDashboard.putNumber("Distance From Tar in Inches", distanceFromPose*39.37007874);
            SmartDashboard.putNumber("Distance From Tar in Meters",distanceFromPose);
        }

}
