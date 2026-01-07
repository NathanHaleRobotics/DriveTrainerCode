package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final Translation2d FL_POS = new Translation2d(Units.inchesToMeters(11), Units.inchesToMeters(12.5));
    public static final Translation2d FR_POS = new Translation2d(Units.inchesToMeters(11), Units.inchesToMeters(-12.5));
    public static final Translation2d BL_POS = new Translation2d(Units.inchesToMeters(-11), Units.inchesToMeters(12.5));
    public static final Translation2d BR_POS = new Translation2d(Units.inchesToMeters(-11), Units.inchesToMeters(-12.5));

    public static final double MAX_LINEAR_SPEED = 3;
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED/Math.hypot(Units.inchesToMeters(11), Units.inchesToMeters(12.5));

    public static final double INTAKE_VOLTS = 5.0;
    public static final double INTAKE_VOLTS_LOWER = 3.0;
}
