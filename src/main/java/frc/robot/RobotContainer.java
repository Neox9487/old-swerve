// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystem.Swerve;

public class RobotContainer {
  private final Joystick joystick;

  private final Swerve swerve;
  
  public RobotContainer() {
    joystick = new Joystick(0);
    swerve = new Swerve();

    configDefaultCommands();
    configKeyBindings();
  }

  private void configDefaultCommands() {
    swerve.setDefaultCommand(
      new RunCommand(
                () -> swerve.drive(
                    joystick.getY() * -1 * Constants.OperatorConstants.DriveSpeed,
                    joystick.getX() * -1 * Constants.OperatorConstants.DriveSpeed,
                    joystick.getRawAxis(4) * -1 * Constants.OperatorConstants.TurnSpeed,
                    true
                ),
                swerve
            )
    );
  }

  private void configKeyBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
