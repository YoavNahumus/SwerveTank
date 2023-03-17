package frc.robot.subsystems;

import static frc.robot.Constants.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.KeepPosition;
import frc.robot.utils.Utils;

/**
 * Represents the chassis of the robot
 */
public class Chassis extends SubsystemBase {
    private final SwerveModule[] modules;
    private final PigeonIMU gyro;
    private final PIDController angleController;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private final Field2d pathDisplay;

    private final double startRoll, startPitch;
    private boolean isBreak;

    /**
     * Creates a new chassis
     */
    public Chassis() {
        modules = new SwerveModule[] {
                new SwerveModule(SwerveModuleConstants.LEFT_FRONT),
                new SwerveModule(SwerveModuleConstants.LEFT_BACK),
                new SwerveModule(SwerveModuleConstants.RIGHT_FRONT),
                new SwerveModule(SwerveModuleConstants.RIGHT_BACK)
        };

        gyro = new PigeonIMU(GYRO_ID);
        gyro.configFactoryDefault();

        angleController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);
        angleController.enableContinuousInput(0, 360);
        angleController.setTolerance(ANGLE_TOLERANCE);

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getGyroRotation(), getModulePositions(),
                new Pose2d(0, 0, getGyroRotation()));

        field = new Field2d();
        pathDisplay = new Field2d();

        startPitch = gyro.getPitch();
        startRoll = gyro.getRoll();

        isBreak = true;

        setupPathDisplay();
    }

    /**
     * Setup the path display
     */
    private void setupPathDisplay() {
        PPSwerveControllerCommand.setLoggingCallbacks((traj) -> {
            pathDisplay.getObject("traj").setTrajectory(traj);
        }, pathDisplay::setRobotPose, null, null);
    }

    /**
     * Gets the angle of the gyro
     * 
     * @return the angle of the gyro
     */
    public double getGyroAngle() {
        return gyro.getFusedHeading();
    }

    /**
     * Gets the rotation of the gyro
     * 
     * @return the rotation of the gyro
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    /**
     * Gets the angle of the robot according to the pose estimator
     * 
     * @return The angle of the robot, between 0 and 360 degrees
     */
    public double getAngle() {
        return Utils.normalizeDegrees(getRotation().getDegrees());
    }

    /**
     * Gets the rotation of the robot according to the pose estimator
     * 
     * @return The rotation of the robot
     */
    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Gets the pose of the robot according to the pose estimator
     * 
     * @return The pose of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the velocity of the robot
     * 
     * @param vx    the x velocity, in meters per second
     * @param vy    the y velocity, in meters per second
     * @param omega the angular velocity, in radians per second
     */
    public void setVelocities(double vx, double vy, double omega) {
        SwerveModuleState[] states = KINEMATICS
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getRotation()));
        setModuleStates(states);
    }

    /**
     * Sets the velocity and the angle of the robot
     * 
     * @param vx    the x velocity, in meters per second
     * @param vy    the y velocity, in meters per second
     * @param angle the angle, in degrees
     */
    public void setAngleVelocities(double vx, double vy, double angle) {
        angleController.setSetpoint(angle);
        double omega = 0;
        if (!angleController.atSetpoint())
            omega = Math.toRadians(angleController.calculate(getAngle(), angle));
        setVelocities(vx, vy, omega);
    }

    /**
     * Sets the module states
     * 
     * @param states the module states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        for (int i = 0; i < 4; i++) {
            states[i] = SwerveModuleState.optimize(states[i], modules[i].getRotation());
            modules[i].setState(states[i]);
        }
    }

    /**
     * Gets the module states
     * 
     * @return the module states
     */
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * Stops the chassis
     */
    public void stop() {
        Arrays.stream(modules).forEach(SwerveModule::stop);
    }

    /**
     * Sets the chassis' neutral mode
     * 
     * @param isBreak whether the chassis should be in break mode or not
     */
    public void setNeutral(boolean isBreak) {
        this.isBreak = isBreak;
        Arrays.stream(modules).forEach(module -> module.setBreak(isBreak));
    }

    /**
     * Swaps the chassis' neutral mode
     */
    public void swapNeutral() {
        setNeutral(!isBreak);
    }

    /**
     * Gets the velocity of the robot
     * 
     * @return the velocity of the robot
     */
    public Translation2d getVelocity() {
        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(getModuleStates());
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(getRotation());
    }

    /**
     * Resets the angle of the gyro
     * 
     * @param angle the angle to reset to
     */
    public void resetAngle(double angle) {
        gyro.setYaw(angle);
        gyro.setFusedHeading(angle);
        Rotation2d rotation = getGyroRotation();
        poseEstimator.resetPosition(rotation, getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
    }

    /**
     * Gets the positions of the modules
     * 
     * @return the positions of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Resets the pose of the robot
     * 
     * @param pose the pose to reset to
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    /**
     * Gets the roll of the robot
     * 
     * @return the roll of the robot
     */
    public double getRoll() {
        return gyro.getRoll() - startRoll;
    }

    /**
     * Gets the pitch of the robot
     * 
     * @return the pitch of the robot
     */
    public double getPitch() {
        return gyro.getPitch() - startPitch;
    }

    /**
     * Gets the up rotation of the robot
     * 
     * @return the up rotation of the robot
     */
    public double getUpRotation() {
        double pitch = getPitch();
        double roll = getRoll();
        double sign;
        if (Math.abs(pitch) > Math.abs(roll))
            sign = Math.signum(pitch);
        else
            sign = Math.signum(roll);
        return sign * Math.hypot(pitch, roll);
    }

    /**
     * Gets the up angular velocity of the robot
     * 
     * @return the up angular velocity of the robot
     */
    public double getUpAngularVel() {
        double[] arr = new double[3];
        gyro.getRawGyro(arr);
        double sign;
        if (Math.abs(arr[0]) > Math.abs(arr[1]))
            sign = Math.signum(arr[0]);
        else
            sign = Math.signum(arr[1]);
        return sign * Math.hypot(arr[0], arr[1]);
    }

    /**
     * Creates a path following command
     * 
     * @param trajectory   the trajectory to follow
     * @param resetPose    whether to reset the pose or not
     * @param keepPosition whether to keep the position or not
     * @return             the path following command
     */
    public Command createPathFollowingCommand(PathPlannerTrajectory trajectory, boolean resetPose,
            boolean keepPosition) {
        return new InstantCommand(() -> {
            if (resetPose)
                setPose(trajectory.getInitialHolonomicPose());
        }).andThen(new PPSwerveControllerCommand(trajectory, this::getPose, KINEMATICS,
                        new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
                        new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
                        new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD),
                        this::setModuleStates, this),
                keepPosition ? new KeepPosition(this, trajectory.getEndState().poseMeters)
                        : new InstantCommand(this::stop));
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), getModulePositions());
        field.setRobotPose(getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("Left Front Module", modules[SwerveModuleConstants.LEFT_FRONT.ordinal()]);
        SmartDashboard.putData("Left Back Module", modules[SwerveModuleConstants.LEFT_BACK.ordinal()]);
        SmartDashboard.putData("Right Front Module", modules[SwerveModuleConstants.RIGHT_FRONT.ordinal()]);
        SmartDashboard.putData("Right Back Module", modules[SwerveModuleConstants.RIGHT_BACK.ordinal()]);

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Path", pathDisplay);

        Utils.addDoubleProperty(builder, "Angle", this::getAngle, 2);
        Utils.addDoubleProperty(builder, "Up Angle", this::getUpRotation, 2);
        Utils.addDoubleProperty(builder, "Up Angular Vel", this::getUpAngularVel, 2);

        Utils.putData("Zero Angle", "Reset", new InstantCommand(() -> resetAngle(0)).ignoringDisable(true));
        Utils.putData("Calirate Offsets", "Calibrate", new InstantCommand(() -> {
            for (SwerveModule module : modules)
                module.calibrateOffset();
        }).ignoringDisable(true));

        Utils.addDoubleProperty(builder, "Velocity", () -> getVelocity().getNorm(), 2);
        Utils.addDoubleProperty(builder, "Heading", () -> getVelocity().getAngle().getDegrees(), 2);

        builder.addBooleanProperty("Is Break", () -> isBreak, this::setNeutral);
    }
}
