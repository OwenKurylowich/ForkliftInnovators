// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.xml.crypto.KeySelector.Purpose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveSubsystem;

public class autonomousCommand extends CommandBase {
  DriveSubsystem subsystem;
  private boolean done = false;
  private AHRS navx;
  private float startYaw = 0;
  
  private boolean firstDriveRun;
  private boolean resetEncoderOne;
  private boolean firstStop;
  private boolean getOffBal;
  private boolean lineUp;
  private boolean secondStop;
  private boolean secondDriveRun;
  private boolean thirdStop;
   
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
    resetEncoderOne = true;
    firstStop = true;
    getOffBal = true;
    lineUp = true;
    secondStop = true;
    secondDriveRun = true;
    thirdStop = true;
    Timer.delay(5); //delay to lean charge station in testing
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(firstDriveRun){
      while(subsystem.autoDrive(1)){} //drive 9 feet//changed to 2 feet
      firstDriveRun = false;
    }
    
     if(resetEncoderOne){subsystem.resetEncoders();}
     if(getOffBal){
       while(subsystem.autoOffBal()){} // get level on ground so it is out of community
       getOffBal = false;
     }
     if(firstStop){
      subsystem.drive(0.0,0.0); //small pause
       Timer.delay(0.1); 
       firstStop = false;
     }
     if(lineUp){
       subsystem.turnToPoint(startYaw); //line up to point straight again
       lineUp = false;
     }
     if(secondStop){
       subsystem.drive(0.0,0.0);
       Timer.delay(0.1); //pause again
       secondStop = false;
     }
     if(secondDriveRun){
       subsystem.autoDrive(-2.5); //drive back up the charge station//made 3.5 from 2.5
       secondDriveRun = false;
     }
     if(thirdStop){
       subsystem.drive(-0.05,-0.05);
       Timer.delay(0.1); //pause again
       thirdStop = false;
     }
     subsystem.toggleBalancePID(); //toggle balance to set some variables for balance
     while(subsystem.autoBal()){SmartDashboard.putNumber("NavX Pitch",navx.getPitch());} //balance
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
