package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim simA = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1.0), DCMotor.getNEO(1));
    private final FlywheelSim simB = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1.0), DCMotor.getNEO(1));
    private final FlywheelSim simC = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1.0), DCMotor.getNEO(1));

    private double appliedVoltsA = 0.0;
    private double appliedVoltsB = 0.0;
    private double appliedVoltsC = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        simA.update(0.02);
        simB.update(0.02);
        simC.update(0.02);

        inputs.motorAPositionRad += simA.getAngularVelocityRadPerSec() * 0.02;
        inputs.motorAVelocityRadPerSec = simA.getAngularVelocityRadPerSec();
        inputs.motorAAppliedVolts = appliedVoltsA;
        inputs.motorACurrentAmps = simA.getCurrentDrawAmps();

        inputs.motorBPositionRad += simB.getAngularVelocityRadPerSec() * 0.02;
        inputs.motorBVelocityRadPerSec = simB.getAngularVelocityRadPerSec();
        inputs.motorBAppliedVolts = appliedVoltsB;
        inputs.motorBCurrentAmps = simB.getCurrentDrawAmps();

        inputs.motorCPositionRad += simC.getAngularVelocityRadPerSec() * 0.02;
        inputs.motorCVelocityRadPerSec = simC.getAngularVelocityRadPerSec();
        inputs.motorCAppliedVolts = appliedVoltsC;
        inputs.motorCCurrentAmps = simC.getCurrentDrawAmps();
    }

    @Override
    public void setMotorAVoltage(double volts) {
        appliedVoltsA = volts;
        simA.setInputVoltage(volts);
    }

    @Override
    public void setMotorBVoltage(double volts) {
        appliedVoltsB = volts;
        simB.setInputVoltage(volts);
    }

    @Override
    public void setMotorCVoltage(double volts) {
        appliedVoltsC = volts;
        simC.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        setMotorAVoltage(0.0);
        setMotorBVoltage(0.0);
        setMotorCVoltage(0.0);
    }
}
