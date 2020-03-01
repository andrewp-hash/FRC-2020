package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;

import static frc.robot.RobotMap.*;

public class DrivetrainSubsystem extends SubsystemBase implements UpdateManager.Updatable {
    private static final double TRACKWIDTH = 17.5;
    private static final double WHEELBASE = 26;

    private static final DrivetrainSubsystem instance;
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable currentPoseTable = nt.getTable("/pathFollowing/current");
    private final NetworkTableEntry currentXEntry = currentPoseTable.getEntry("x");
    private final NetworkTableEntry currentYEntry = currentPoseTable.getEntry("y");
    private final NetworkTableEntry currentAngleEntry = currentPoseTable.getEntry("angle");

    private final double X_SCALE = 1;
    private final double Y_SCALE = 1;

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleEncoder(new AnalogInput(DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER),
                            DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET)
                    .angleMotor(new CANSparkMax(DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(new CANSparkMax(DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                    .build();
    private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

    private final SwerveKinematics kinematics = new SwerveKinematics(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front
                                                                                                                      // Left
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front Right
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Back Left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back Right
    );
    private final SwerveOdometry odometry = new SwerveOdometry(kinematics,
            new RigidTransform2(new Vector2(TRACKWIDTH / 2, WHEELBASE / 2), Rotation2.fromDegrees(90)));

    private final Object sensorLock = new Object();
    private final NavX navX$sensorLock = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    private RigidTransform2 pose$kinematicsLock = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    private HolonomicDriveSignal driveSignal$stateLock = null;

    private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    static {
        instance = new DrivetrainSubsystem();
    }

    private DrivetrainSubsystem() {
        synchronized (sensorLock) {
            navX$sensorLock.setInverted(true);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(1, 0).withSize(2, 3);
        moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(3, 0).withSize(2, 3);
        moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(5, 0).withSize(2, 3);
        moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

        ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(7, 0).withSize(2, 3);
        moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    /**
     * @return The current rate of change in yaw angle (in degrees per second)
     */
    public double getAngularVelocity() {
        synchronized (sensorLock) {
            return navX$sensorLock.getRate();
        }
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose$kinematicsLock;
        }
    }

    public RigidTransform2 getScaledPose() {
        final var pose = getPose();
        final var translation = pose.translation.multiply(X_SCALE, Y_SCALE);
        final var rotation = pose.rotation.rotateBy(Rotation2.fromRadians(Math.PI / 2));

        return new RigidTransform2(translation, rotation);
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        synchronized (stateLock) {
            driveSignal$stateLock = new HolonomicDriveSignal(translationalVelocity, -rotationalVelocity, fieldOriented);
        }
    }

    public void resetPose(Vector2 translation, Rotation2 angle) {
        System.out.println("Reset Pose");
        synchronized (kinematicsLock) {
            odometry.resetPose(
                    new RigidTransform2(new Vector2(translation.x / X_SCALE, translation.y / Y_SCALE), angle));
            resetGyroAngle(angle);
            pose$kinematicsLock = odometry.getPose();
        }
        updatePoseNT();
    }

    public void resetGyroAngle(Rotation2 angle) {
        System.out.println("Reset Gyro Angle");
        synchronized (sensorLock) {
            navX$sensorLock.setAdjustmentAngle(navX$sensorLock.getUnadjustedAngle().rotateBy(angle.inverse()));
        }
    }

    @Override
    public void update(double timestamp, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = driveSignal$stateLock;
        }

        updateModules(driveSignal, dt);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle()))
                    .scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX$sensorLock.getAngle();
        }

        RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

        synchronized (kinematicsLock) {
            pose$kinematicsLock = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal signal, double dt) {
        ChassisVelocity velocity;
        if (signal == null) {
            velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (signal.isFieldOriented()) {
            velocity = new ChassisVelocity(signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    signal.getRotation());
        } else {
            velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
        }

        Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    private void updatePoseNT() {
        var pose = getScaledPose();

        currentAngleEntry.setDouble(pose.rotation.toRadians());
        currentXEntry.setDouble(pose.translation.x);
        currentYEntry.setDouble(pose.translation.y);

    }

    @Override
    public void periodic() {
        updatePoseNT();
        // poseXEntry.setDouble(pose.translation.x);
        // poseYEntry.setDouble(pose.translation.y);
        // poseAngleEntry.setDouble(pose.rotation.toDegrees());

        // for (int i = 0; i < modules.length; i++) {
        // var module = modules[i];
        // moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
        // }
    }
}
