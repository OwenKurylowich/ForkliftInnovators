// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private DriveSubsystem subsystem;
  private double leftValue;
  private double rightValue;
  /** Creates a new DriveCommand. */
  public DriveCommand(double leftValue, double rightValue) {
    this.subsystem = DriveSubsystem.getInstance();
        this.leftValue = leftValue;
        this.rightValue = rightValue;
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
    subsystem.drive(leftValue, rightValue);
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
