// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.ToggleBalanceCommand;
import frc.robot.Subsystems.DriveSubsystem;

public class RobotContainer {

  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  private final Joystick controller = new Joystick(2);

  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final DriveCommand driveCommand = new DriveCommand(leftJoystick, rightJoystick);
    private final ToggleBalanceCommand toggleBalance = new ToggleBalanceCommand();

    private final JoystickButton balanceButton = new JoystickButton(controller, 2);
    

  public RobotContainer() {
    configureBindings();

    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, driveCommand);
  }

  private void configureBindings() {
    balanceButton.toggleOnTrue(toggleBalance);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
