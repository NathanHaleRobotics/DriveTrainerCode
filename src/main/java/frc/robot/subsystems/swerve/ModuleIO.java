package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;



public interface ModuleIO {
    
    @AutoLog
    class ModuleIOInputs {
        public double steerPos = 0;
        public double steerVel = 0;
        public double steerAppliedVolts = 0;
        public double steerCurrent = 0;
        public double steerTemperature = 0;

        public double drivePos = 0;
        public double driveVel = 0;
        public double driveAppliedVolts = 0;
        public double driveCurrent = 0;
        public double driveTemperature = 0;

    }
    
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVel(double velocityRadPerSec){}

    public default void setSteerPos(double rotationRad){}

    public default int getID(){return -1;}
    
    //woke
}

