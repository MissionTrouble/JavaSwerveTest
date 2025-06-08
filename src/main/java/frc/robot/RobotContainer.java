// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.swerve.commands.SwerveJoystickCmd;
import frc.robot.subsystems.swerve.SwerveRobot;

public class RobotContainer {
  
    private final SwerveRobot swerveSubsystem = new SwerveRobot();

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 4).onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.zeroHeading()));
    }

    public Command getAutonomousCommand() {
        return null; // Add autonomous command here
    }

    public SwerveRobot getSwerveSubsystem() {
            return swerveSubsystem;
    }
}