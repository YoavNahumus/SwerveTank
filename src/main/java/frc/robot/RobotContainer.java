// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
    private static RobotContainer instance;

    public final CommandXboxController main = new CommandXboxController(0);
    public final CommandXboxController secondary = new CommandXboxController(1);

    private final Chassis chassis;

    private RobotContainer() {
        chassis = new Chassis();
        chassis.setDefaultCommand(new Drive(chassis, main));
        SmartDashboard.putData("Chassis", chassis);

        configureBindings();
    }


    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
