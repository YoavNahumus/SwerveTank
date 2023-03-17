package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ControllerSide;

/**
 * Drives the robot
 */
public class Drive extends CommandBase {
    private final Chassis chassis;
    private final CommandXboxController controller;
    private final SlewRateLimiter xSlew;
    private final SlewRateLimiter ySlew;

    /**
     * Creates a new Drive command
     * 
     * @param chassis    The chassis to use
     * @param controller The controller to use
     */
    public Drive(Chassis chassis, CommandXboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
        xSlew = new SlewRateLimiter(Constants.DRIVE_RATE_LIMIT);
        ySlew = new SlewRateLimiter(Constants.DRIVE_RATE_LIMIT);

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        boolean red = DriverStation.getAlliance() == Alliance.Red;
        Translation2d xy = Utils.getStick(controller.getHID(), ControllerSide.LEFT, true).times(red ? -1 : 1);
        double vx = xSlew.calculate(xy.getY()) * ChassisConstants.MAX_DRIVE_SPEED;
        double vy = ySlew.calculate(-xy.getX()) * ChassisConstants.MAX_DRIVE_SPEED;
        double vo = Utils.getScaledTriggerDiff(controller.getHID(), ControllerSide.LEFT, 2)
                * ChassisConstants.MAX_ANGULAR_SPEED;
        Rotation2d angle = Utils.getStickRotation(controller.getHID(), ControllerSide.RIGHT);
        if (angle == null && vx == 0 && vy == 0)
            chassis.stop();
        else if (angle == null)
            chassis.setVelocities(vx, vy, vo);
        else
            chassis.setAngleVelocities(vx, vy, angle.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
