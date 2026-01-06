package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ModuleIOSpark implements ModuleIO{
    
        
    private SparkMax steer;
    private SparkMax drive;

    private double steerSet = 0;
    private double driveSet = 0;
    //4.5 0 0.07
    private ProfiledPIDController steerPID = new ProfiledPIDController(3.6, 0, 0.09, new Constraints(23 , 350));
    private SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(0.11, 0.07, 0.12);
    private PIDController drivePID = new PIDController(0, 0, 0.0);
    // 0 1/6.75
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 1.0/6.75);

    

    public ModuleIOSpark(int sid, int did){
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
        steer = new SparkMax(sid, MotorType.kBrushless);
        drive = new SparkMax(did, MotorType.kBrushless);

        var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        // .positionConversionFactor(DRIVE_ENCODER_POS_FACTOR)
        // .velocityConversionFactor(DRIVE_ENCODER_VEL_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(4);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 50))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        drive,
        5,
        () ->
            drive.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(drive, 5, () -> drive.getEncoder().setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(2*Math.PI)
        .velocityConversionFactor(2*Math.PI/60)
        .averageDepth(8);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kNoSensor);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / 50))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        steer,
        5,
        () ->
            steer.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    public void updateInputs(ModuleIOInputs in){
        double pos = steer.getAbsoluteEncoder().getPosition() - Math.PI;
        double vel = drive.getEncoder().getVelocity();

        in.steerPos = pos;
        in.steerVel = steer.getAbsoluteEncoder().getVelocity();
        in.steerAppliedVolts = steer.getAppliedOutput()*steer.getBusVoltage();
        in.steerCurrent = steer.getOutputCurrent();
        in.steerTemperature = steer.getMotorTemperature();
        
        in.driveVel = vel;
        in.drivePos = drive.getEncoder().getPosition();
        in.driveAppliedVolts = drive.getAppliedOutput()*drive.getBusVoltage();
        in.driveCurrent = drive.getOutputCurrent();
        in.driveTemperature = drive.getMotorTemperature();
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/positionSet", steerPID.getSetpoint().position);
        
        double goSteer = steerPID.calculate(pos, steerSet) + steerFF.calculate(steerPID.getSetpoint().velocity);
        double goDrive = drivePID.calculate(vel, -driveSet) + driveFF.calculate(-driveSet);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOuts", goSteer);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOutd", goDrive);
        
        
        steer.setVoltage(goSteer);
        drive.setVoltage(goDrive);
        
    }
    public void setSteerPos(double point){
        steerSet = point;
    }
    public void setDriveVel(double point){
        driveSet = point;
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

    
    public int getID(){
        return steer.getDeviceId();
    }
}
