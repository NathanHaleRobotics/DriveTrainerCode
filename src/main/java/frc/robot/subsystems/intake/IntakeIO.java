package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorAPositionRad = 0.0;
        public double motorAVelocityRadPerSec = 0.0;
        public double motorAAppliedVolts = 0.0;
        public double motorACurrentAmps = 0.0;
        public double motorATempCelcius = 0.0;

        public double motorBPositionRad = 0.0;
        public double motorBVelocityRadPerSec = 0.0;
        public double motorBAppliedVolts = 0.0;
        public double motorBCurrentAmps = 0.0;
        public double motorBTempCelcius = 0.0;

        public double motorCPositionRad = 0.0;
        public double motorCVelocityRadPerSec = 0.0;
        public double motorCAppliedVolts = 0.0;
        public double motorCCurrentAmps = 0.0;
        public double motorCTempCelcius = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setMotorAVoltage(double volts) {}

    public default void setMotorBVoltage(double volts) {}

    public default void setMotorCVoltage(double volts) {}

    public default void stop() {}
}
