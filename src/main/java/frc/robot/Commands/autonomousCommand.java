// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.xml.crypto.KeySelector.Purpose;

import com.kauailabs.navx.frc.AHRS;

import driveforward.autoDrivePID;
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
  private boolean intitialStop;
  private boolean initialStop;
  private boolean test;
  private boolean atSetpoint;
  private float endYaw;
   
  /** Creates a new autonomousCommand. */
  public autonomousCommand() {
    subsystem = DriveSubsystem.getInstance();
    navx = subsystem.getNavx();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialStop = true;
    subsystem.resetEncoders();
    subsystem.brakeMode(true);
    startYaw = navx.getYaw();
    atSetpoint = false;
    firstDriveRun = true;
    resetEncoderOne = true;
    firstStop = true;
    getOffBal = true;
    lineUp = true;
    test = true;
    secondStop = true;
    secondDriveRun = true;
    thirdStop = true;
    subsystem.setCalculatedPower(1);
    subsystem.setBalanceTime(12);
    subsystem.toggleBalancePID();
    subsystem.resetEncoders();
   Timer.delay(1); //delay to lean charge station in testing
  }

   //Called every time the scheduler runs while the command is scheduled.
  @Override
 public void execute() {
  
  //while(firstDriveRun){
   // while(subsystem.autoDrive(6)){};
    //subsystem.getTurnPID();
    //checking the value of atSetpoint = false then autoBal
   
    //  subsystem.setCalculatedPower(1);
    //  subsystem.setBalanceTime(12);
     //subsystem.setBrakeModeOne(getOffBal);
     //subsystem.tankDriveVolts(3, 3);
  
      // subsystem.tankDriveVolts(3, 3);
      // subsystem.tankDriveVolts(5, 5);
      // subsystem.tankDriveVolts(9, 9);

  //}
   // subsystem.toggleBalancePID();
   // subsystem.resetEncoders();

  
// if(initialStop){
//    subsystem.turnToPoint(startYaw);
//    while(subsystem.turnToPoint(startYaw)){
//   Timer.delay(0.1);
//     subsystem.brakeMode(initialStop);
//     subsystem.getTurnPID();
//     subsystem.resetEncoders();
//    }
//   initialStop = false;
//    }

//     if(firstDriveRun){
//     while(subsystem.autoDrive(6)){} //drive 9 feet//changed from 4 to 6 feet//
//     subsystem.resetEncoders();
//       firstDriveRun = false;
//    }
    
//      if(resetEncoderOne){ resetEncoderOne = false;
//      subsystem.resetEncoders();}
//     if(getOffBal){
//        while(subsystem.autoOffBal()){} // get level on ground so it is out of community
//        getOffBal = false;
//      }
//      if(firstStop){
//       subsystem.drive(0.0,0.0); //small pause
//        Timer.delay(0.1); 
//        subsystem.getTurnPID();
//        subsystem.resetEncoders();//added this and the thing above it//
//        firstStop = false;
//      }
//      if(lineUp){
//        subsystem.turnToPoint(startYaw); //line up to point straight again
//      lineUp = false;
//     }
//     if(secondStop){
//        subsystem.drive(0.0,0.0);
//     Timer.delay(0.1); //pause again
//        secondStop = false;
//      }
//      if(secondDriveRun){
//        subsystem.autoDrive(-3.5); //drive back up the charge station//changed to 2.5 to 3.5//
//        secondDriveRun = false;
//      }
//      if(thirdStop){
//        subsystem.drive(-0.05,-0.05);
//      Timer.delay(0.1); //pause again
//        thirdStop = false;
//      }
//      subsystem.toggleBalancePID(); //toggle balance to set some variables for balance
//      while(subsystem.autoBal()){SmartDashboard.putNumber("NavX Pitch",navx.getPitch());} //balance
//  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.autoDrive(9);
  }
}
