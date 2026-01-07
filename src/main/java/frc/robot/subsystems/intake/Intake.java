package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    public Command runMotorA(double volts) {
        return this.runEnd(
            () -> io.setMotorAVoltage(volts),
            () -> io.setMotorAVoltage(0.0)
        );
    }

    public Command runMotorB(double volts) {
        return this.runEnd(
            () -> io.setMotorBVoltage(volts),
            () -> io.setMotorBVoltage(0.0)
        );
    }

    public Command runMotorC(double volts) {
        return this.runEnd(
            () -> io.setMotorCVoltage(volts),
            () -> io.setMotorCVoltage(0.0)
        );
    }

    public Command runAll(double volts) {
        return this.runEnd(
            () -> {
                io.setMotorAVoltage(volts);
                io.setMotorBVoltage(volts);
                io.setMotorCVoltage(volts);
            },
            () -> {
                io.setMotorAVoltage(0.0);
                io.setMotorBVoltage(0.0);
                io.setMotorCVoltage(0.0);
            }
        );
    }
    public Command intake() {
        return this.runEnd(
            () -> {
                io.setMotorAVoltage(Constants.INTAKE_VOLTS);
                io.setMotorBVoltage(Constants.INTAKE_VOLTS);
                io.setMotorCVoltage(Constants.INTAKE_VOLTS_LOWER);
            },
            () -> {
                io.setMotorAVoltage(0.0);
                io.setMotorBVoltage(0.0);
                io.setMotorCVoltage(0.0);
            }
        );
    }

    public Command outtake() {
        return this.runEnd(
            () -> {
                io.setMotorAVoltage(-Constants.INTAKE_VOLTS);
                io.setMotorBVoltage(-Constants.INTAKE_VOLTS);
                io.setMotorCVoltage(-Constants.INTAKE_VOLTS_LOWER);
            },
            () -> {
                io.setMotorAVoltage(0.0);
                io.setMotorBVoltage(0.0);
                io.setMotorCVoltage(0.0);
            }
        );
    }

    public Command stop() {
        return this.runOnce(() -> {
            io.setMotorAVoltage(0.0);
            io.setMotorBVoltage(0.0);
            io.setMotorCVoltage(0.0);
        });
    }
}
