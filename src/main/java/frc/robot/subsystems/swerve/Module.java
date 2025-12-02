package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
    
    private SparkMax steer;
    private SparkMax drive;

    private double steerSet = 0;
    private double driveSet = 0;

    private PIDController steerPID = new PIDController(4.5, 0, 0.07);
    private PIDController drivePID = new PIDController(0, 0, 0.0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 1/6.75);

    

    public Module(int sid, int did){
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
        steer = new SparkMax(sid, MotorType.kBrushless);
        drive = new SparkMax(did, MotorType.kBrushless);

        var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
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

    public void periodic(){
        double pos = steer.getAbsoluteEncoder().getPosition() - Math.PI;
        double vel = drive.getEncoder().getVelocity();
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/position", pos);
        Logger.recordOutput("subsystems/drive"+steer.getDeviceId()+"/positionSet", steerSet);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/speed", vel);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/speedSet", -driveSet);
        
        double goSteer = steerPID.calculate(pos, steerSet);
        double goDrive = drivePID.calculate(vel, -driveSet) + driveFF.calculate(-driveSet);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOuts", goSteer);
        Logger.recordOutput("subsystems/drive/"+steer.getDeviceId()+"/PIDOutd", goDrive);
        
        steer.setVoltage(goSteer);
        drive.setVoltage(goDrive);
        
    }
    public void setSteerPoint(double point){
        steerSet = point;
    }
    public void setDrivePoint(double point){
        driveSet = point;
    }
    public void setState(SwerveModuleState state){
        state.optimize(new Rotation2d(steer.getAbsoluteEncoder().getPosition() - Math.PI));
        state.cosineScale(new Rotation2d(steer.getAbsoluteEncoder().getPosition() - Math.PI));
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
