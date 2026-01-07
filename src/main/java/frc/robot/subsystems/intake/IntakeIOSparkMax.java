package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
    private final SparkMax motorA;
    private final SparkMax motorB;
    private final SparkMax motorC;

    public IntakeIOSparkMax(int motorAId, int motorBId, int motorCId) {
        motorA = new SparkMax(motorAId, MotorType.kBrushless);
        motorB = new SparkMax(motorBId, MotorType.kBrushless);
        motorC = new SparkMax(motorCId, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        motorA.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorB.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorC.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorAPositionRad = Units.rotationsToRadians(motorA.getEncoder().getPosition());
        inputs.motorAVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motorA.getEncoder().getVelocity());
        inputs.motorAAppliedVolts = motorA.getAppliedOutput() * motorA.getBusVoltage();
        inputs.motorACurrentAmps = motorA.getOutputCurrent();
        inputs.motorATempCelcius = motorA.getMotorTemperature();

        inputs.motorBPositionRad = Units.rotationsToRadians(motorB.getEncoder().getPosition());
        inputs.motorBVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motorB.getEncoder().getVelocity());
        inputs.motorBAppliedVolts = motorB.getAppliedOutput() * motorB.getBusVoltage();
        inputs.motorBCurrentAmps = motorB.getOutputCurrent();
        inputs.motorBTempCelcius = motorB.getMotorTemperature();

        inputs.motorCPositionRad = Units.rotationsToRadians(motorC.getEncoder().getPosition());
        inputs.motorCVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motorC.getEncoder().getVelocity());
        inputs.motorCAppliedVolts = motorC.getAppliedOutput() * motorC.getBusVoltage();
        inputs.motorCCurrentAmps = motorC.getOutputCurrent();
        inputs.motorCTempCelcius = motorC.getMotorTemperature();
    }

    @Override
    public void setMotorAVoltage(double volts) {
        motorA.setVoltage(volts);
    }

    @Override
    public void setMotorBVoltage(double volts) {
        motorB.setVoltage(volts);
    }

    @Override
    public void setMotorCVoltage(double volts) {
        motorC.setVoltage(volts);
    }

    @Override
    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
        motorC.stopMotor();
    }
}
