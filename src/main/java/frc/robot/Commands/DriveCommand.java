// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private DriveSubsystem subsystem;
  private Joystick leftJoystick;
  private Joystick rightJoystick;
  /** Creates a new DriveCommand. */
  public DriveCommand(Joystick leftJoystick, Joystick rightJoystick) {
    this.subsystem = DriveSubsystem.getInstance();
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
        subsystem.setDefaultCommand(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.drive(leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
