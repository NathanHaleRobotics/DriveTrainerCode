package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.*;

public class DriveSubsystem extends SubsystemBase{
    
    private Module[] modules;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
    private ChassisSpeeds targetSpeed = new ChassisSpeeds(0, 0, 0);
    private Pigeon2 bird = new Pigeon2(30);

    public DriveSubsystem(Module... mods){
        this.modules = mods;
    }

    @Override
    public void periodic(){
        SwerveModuleState[] s = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeed, bird.getRotation2d()));
        modules[0].setState(s[0]);
        modules[1].setState(s[1]);
        modules[2].setState(s[2]);
        modules[3].setState(s[3]);
        Logger.recordOutput("drive/setStates", s);
        Logger.recordOutput("drive/angle", bird.getRotation2d());
        if (DriverStation.isDisabled()) {
            for (Module m  : modules){
                m.setSteerPoint(m.getPos());
            }
        }
        for(Module m : modules){
            m.periodic();
        }
        
    }

    public void drive(ChassisSpeeds s){
        targetSpeed = s;
    }
    public void zero(){
        bird.reset();
    }
}
