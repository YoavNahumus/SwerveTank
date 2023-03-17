package frc.robot.subsystems;

import static frc.robot.Constants.SwerveModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Utils;

/**
 * Represents a swerve module of a swerve-tank drive
 */
public class SwerveModule extends SubsystemBase {
    private final TalonFX left, right;
    private final CANCoder encoder;
    private double offset;

    private final PIDController angleController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);

    private double velocity;
    private double angle;

    private boolean stop;

    /**
     * Creates a new swerve module
     * 
     * @param constants the constants of the module
     */
    public SwerveModule(SwerveModuleConstants constants) {
        left = new TalonFX(constants.leftID);
        right = new TalonFX(constants.rightID);
        encoder = new CANCoder(constants.encoderID);
        offset = constants.encoderOffset;

        angle = getAngle();
        velocity = 0;

        angleController.enableContinuousInput(0, 360);

        configureDevices();
    }

    /**
     * Configures the devices
     */
    private void configureDevices() {
        left.configFactoryDefault();
        right.configFactoryDefault();
        encoder.configFactoryDefault();

        left.setInverted(true);
        right.setInverted(false);

        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);

        left.config_kP(0, VELOCITY_KP);
        left.config_kI(0, VELOCITY_KI);
        left.config_kD(0, VELOCITY_KD);
        right.config_kP(0, VELOCITY_KP);
        right.config_kI(0, VELOCITY_KI);
        right.config_kD(0, VELOCITY_KD);
    }

    /**
     * Sets the velocity of the module
     * 
     * @param velocity the velocity to set, in meters per second
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
        stop = false;
    }

    /**
     * Sets the angle of the module
     * 
     * @param angle the angle to set, in degrees
     */
    public void setAngle(double angle) {
        this.angle = angle;
        stop = false;
    }

    /**
     * Gets the velocity of the module
     * 
     * @return the velocity of the module, in meters per second
     */
    public double getVelocity() {
        return ((left.getSelectedSensorVelocity() + right.getSelectedSensorVelocity()) / 2) * 10 / COUNTES_PER_METER;
    }

    /**
     * Gets the angle of the module
     * 
     * @return the angle of the module, in degrees
     */
    public double getAngle() {
        return Utils.normalizeDegrees(encoder.getAbsolutePosition() - offset);
    }

    /**
     * Gets the rotation of the module
     * 
     * @return the rotation of the module
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Gets the state of the module
     * 
     * @return the state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle));
    }

    /**
     * Gets the distance of the module
     * 
     * @return the distance of the module, in meters
     */
    public double getDistance() {
        return ((left.getSelectedSensorPosition() + right.getSelectedSensorPosition()) / 2) / COUNTES_PER_METER;
    }

    /**
     * Gets the position of the module
     * 
     * @return the position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Sets the state of the module
     * 
     * @param state the state to set
     */
    public void setState(SwerveModuleState state) {
        this.velocity = state.speedMetersPerSecond;
        this.angle = state.angle.getDegrees();
        stop = false;
    }

    /**
     * Stops the module
     */
    public void stop() {
        stop = true;
        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the neutral mode of the module
     * 
     * @param isBreak whether the module should be in brake mode
     */
    public void setBreak(boolean isBreak) {
        left.setNeutralMode(isBreak ? NeutralMode.Brake : NeutralMode.Coast);
        right.setNeutralMode(isBreak ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Calibrates the offset of the module
     */
    public void calibrateOffset() {
        offset = encoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {
        if (stop) return;

        double angleOutput = Math.toRadians(angleController.calculate(getAngle(), angle));

        DifferentialDriveWheelSpeeds speeds = KINEMATICS.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angleOutput));
        speeds.desaturate(MAX_VELOCITY);

        left.set(ControlMode.Velocity, speeds.leftMetersPerSecond * COUNTES_PER_METER / 10,
                DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(speeds.leftMetersPerSecond));

        right.set(ControlMode.Velocity, speeds.rightMetersPerSecond * COUNTES_PER_METER / 10,
                DemandType.ArbitraryFeedForward, FEEDFORWARD.calculate(speeds.rightMetersPerSecond));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        Utils.addDoubleProperty(builder, "Angle", this::getAngle, 2);
        builder.addDoubleProperty("Velocity", this::getVelocity, this::setVelocity);
        builder.addDoubleProperty("Offset", () -> offset, null);
        builder.addDoubleProperty("Wanted Angle", () -> angle, this::setAngle);
        builder.addDoubleProperty("Wanted Velocity", () -> velocity, this::setVelocity);
    }
}
