// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveSubsystem;

public class autonomousCommand extends CommandBase {
  DriveSubsystem subsystem;
  private boolean autoDriveDone= false;
  private boolean done = false;
  private AHRS navx;
  private float startYaw = 0;
  
  private boolean firstDriveRun = true;
  private boolean firstStop = true;
   
  /** Creates a new autonomousCommand. */
  public autonomousCommand() {
    subsystem = DriveSubsystem.getInstance();
    navx = subsystem.getNavx();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetEncoders();
    subsystem.brakeMode(true);
    startYaw = navx.getYaw();
    firstDriveRun = true;
    firstStop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if(firstDriveRun){
      while(subsystem.autoDrive(4)){} //drive 4 feet
      firstDriveRun = false;
    }
    if(firstStop){
      subsystem.drive(0.05,0.05);
      Timer.delay(0.1);
      firstStop = false;
    }
    subsystem.toggleBalancePID(); //toggle balance to set some variables for balance
    while(subsystem.autoBal()){SmartDashboard.putNumber("NavX Pitch",navx.getPitch());} //balance
    Timer.delay(2);
    while(subsystem.turnToPoint(startYaw)){} //rotate back to start angle
    while(subsystem.autoBal()){SmartDashboard.putNumber("NavX Pitch",navx.getPitch());}//balance again
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
