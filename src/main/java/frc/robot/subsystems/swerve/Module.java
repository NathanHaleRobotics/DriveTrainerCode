package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
    ModuleIO mod;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io){
        this.mod = io;
    }

    public void periodic(){
        mod.updateInputs(inputs);
        Logger.processInputs("subsystems/drive/"+mod.getID(), inputs);
        
    }
    public void setSteerPoint(double point){
        Logger.recordOutput("subsystems/drive/"+mod.getID()+"/positionGoal", point);
        mod.setSteerPos(point);
    }
    public void setDrivePoint(double point){
        Logger.recordOutput("subsystems/drive/"+mod.getID()+"/speedGoal", point);
        mod.setDriveVel(point);
    }
    public double getSteerPos(){
        return inputs.steerPos;
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(inputs.drivePos , new Rotation2d(inputs.steerPos));
    }

    public void setState(SwerveModuleState state){
        state.optimize(new Rotation2d(inputs.steerPos));
        state.cosineScale(new Rotation2d(inputs.steerPos));
        setSteerPoint(state.angle.getRadians());
        setDrivePoint(state.speedMetersPerSecond/Units.inchesToMeters(2));
    }

    
    /** Attempts to run the command until no error is produced. */
    public void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            }
        }
    }
}
