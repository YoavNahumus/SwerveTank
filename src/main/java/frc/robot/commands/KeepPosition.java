package frc.robot.commands;

import static frc.robot.Constants.ChassisConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/**
 * Keeps the robot at a certain position
 */
public class KeepPosition extends CommandBase {
    private final Chassis chassis;
    private final Pose2d position;
    private final PIDController xController, yController, thetaController;

    /**
     * Creates a new KeepPosition command
     * 
     * @param chassis  The chassis to use
     * @param position The position to keep
     */
    public KeepPosition(Chassis chassis, Pose2d position) {
        this.chassis = chassis;
        this.position = position;

        xController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KP);
        yController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KP);
        thetaController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);
        thetaController.enableContinuousInput(0, 360);

        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);
        thetaController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(position.getTranslation().getX());
        yController.setSetpoint(position.getTranslation().getY());
        thetaController.setSetpoint(position.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();
        double vx = 0, vy = 0, vo = 0;
        if (!xController.atSetpoint())
            vx = xController.calculate(currentPose.getX());
        if (!yController.atSetpoint())
            vy = yController.calculate(currentPose.getY());
        if (!thetaController.atSetpoint())
            vo = thetaController.calculate(currentPose.getRotation().getDegrees());

        chassis.setVelocities(vx, vy, vo);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
