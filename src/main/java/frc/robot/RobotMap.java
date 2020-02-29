package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotMap {
    // On the practice bot, there should be an empty file at /home/lvuser/isPractice
    private static final File isPracticeFile = new File(
            Filesystem.getOperatingDirectory().toPath().resolve("isPractice").toString());
    public static final boolean isPractice = isPracticeFile.exists();

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER = 3;
    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR = 7;
    public static final double DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET = Math.toRadians(isPractice ? -51 : -108);
    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;

    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR = 3;
    public static final double DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET = Math.toRadians(isPractice ? -52 : 152);
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;

    public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER = 2;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR = 5;
    public static final double DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET = Math.toRadians(isPractice ? 189 : -35);
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR = 6;

    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER = 0;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR = 1;
    public static final double DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET = Math.toRadians(isPractice ? 152 : 95);
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;

    public static final int UPPER_SHOOTER_MOTOR = 9;
    public static final int LOWER_SHOOTER_MOTOR = 10;
    public static final int INTAKE_MOTOR = 11;
    public static final int INDEXER_FRONT_MOTOR = 12;
    public static final int INDEXER_BACK_MOTOR = 13;
    public static final int CLIMB_MOTOR = 14;
    public static final int SPINNER_MOTOR = 15;

    public static final int CLIMBER_AIR_IN = 0;
    public static final int CLIMBER_AIR_OUT = 1;
    public static final int SPINNER_AIR_IN = 2;
    public static final int SPINNER_AIR_OUT = 3;
    public static final int INTAKE_RIGHT_AIR_IN = 4;
    public static final int INTAKE_RIGHT_AIR_OUT = 5;
    public static final int INTAKE_LEFT_AIR_IN = 6;
    public static final int INTAKE_LEFT_AIR_OUT = 7;

    public static final int BALL_SENSOR_UPPER = 4;
    public static final int BALL_SENSOR_LOWER = 5;
}
