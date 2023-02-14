// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Localization.ForkliftOdometry;

public class DriveSubsystem extends SubsystemBase {



 
  private final WPI_TalonSRX frontRight = new WPI_TalonSRX(Constants.frontRightCAN);
  private final WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.backRightCAN);
  private final WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.frontLeftCAN);
  private final WPI_TalonSRX backLeft = new WPI_TalonSRX(Constants.backLeftCAN);

  private static DriveSubsystem instance = null;

  private final MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);
  private final MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  public static RamseteController controller = new RamseteController(Constants.Drive.kRamseteB, Constants.Drive.kRamseteZeta);

    public static PIDController ramsetePIDController = new PIDController(Constants.Drive.kPDriveVel, 0, 0);
    AHRS navx = new AHRS(SerialPort.Port.kMXP);
    private double navxResetOffsetX;
    private double navxResetOffsetY;
    private double navxResetOffsetRot;
    DifferentialDriveOdometry odometry;
    ForkliftOdometry forkliftOdometry;

    private boolean BALANCING = false;
    private final double bkP = 0.038;   //0.0325 for no extrsa weight, 0.04 for extra weight, 0.045 with max weight
    private final double bkI = 0.0;   //0.14 for no extra weight, 0.15 for extra weight,0.018 with max weight
    private final double bkD = 0.01;   // 0.011 for normal and extra wweight, 
    private final double gyroSetpointAngle = 0;
    private final PIDController balancePID;
    private final double tkP = 0.01;
    private final double tkI = 0.01;
    private final double tkD = 0.01;
    private final PIDController turnPID;
    private final double akP = 0.1;
    private final double akI = 0.0;
    private final double akD = 0.01;
    private final PIDController autoDrivePID;
    private double calculatedPower = 0;
    private double turnCalc = 0;
    private double balanceTime = 0;
    private boolean brakeMode = false;
    private float startYaw = 0;
    private float endYaw = 0;
    private float yawLeftError = 0;
    private float yawRightError = 0;



  /** Creates a new Drive. */
  public DriveSubsystem() {

    right.setInverted(true);
    frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);

    navx.reset();
    navx.resetDisplacement();
    balancePID = new PIDController(bkP, bkI, bkD);
    balancePID.setTolerance(2.0);
    balancePID.enableContinuousInput(-180, 180);
    turnPID = new PIDController(tkP, tkI, tkD);
    turnPID.setTolerance(2);
    turnPID.enableContinuousInput(-180, 180);
    autoDrivePID = new PIDController(akP, akI, akD);
    autoDrivePID.setTolerance(150);
    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition());
        forkliftOdometry = new ForkliftOdometry(Constants.Drive.kTrackwidthMeters,
                Constants.Drive.kLongitudinalDistance,
                frontLeft,
                frontRight
                );
  }

  public static DriveSubsystem getInstance() {
    if (instance == null) {
        instance = new DriveSubsystem();
    }
    return instance;
}

public Pose2d getPoseFromNavX() {
  return new Pose2d(
          navx.getDisplacementX() + navxResetOffsetX,
          navx.getDisplacementY() + navxResetOffsetY,
          new Rotation2d(navx.getRotation2d().getRadians() + navxResetOffsetRot)
  );
}

public Pose2d getPoseFromOdometry() {
  Pose2d pose = forkliftOdometry.calculate();
  return pose;
}

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(frontLeft.getSelectedSensorVelocity(), frontRight.getSelectedSensorVelocity());
}

public void resetEncoders() {
  frontLeft.setSelectedSensorPosition(0);
  frontRight.setSelectedSensorPosition(0);
}

public void tankDriveVolts(double leftVolts, double rightVolts) {
  left.setVoltage(leftVolts);
  right.setVoltage(rightVolts);
  drive.feed();
}
public void resetOdometry(Pose2d pose) {
  resetEncoders();
  odometry.resetPosition(navx.getRotation2d(), frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition(), pose);
}
public void resetNavX(Pose2d pose) {
  navxResetOffsetX = pose.getX();
  navxResetOffsetY = pose.getY();
  navxResetOffsetRot = pose.getRotation().getRadians();
  resetEncoders(); //in case that affects the getRate() function of the encoders
}
public double getAverageEncoderDistance() {
  return (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2.0;
}

public void toggleBalancePID() {
  BALANCING = (BALANCING == false);
  balanceTime = 0;
  startYaw = navx.getYaw();
  if (BALANCING){
    endYaw = startYaw + 90;
    if (endYaw > 180)
      endYaw = ((endYaw-180)*2)-endYaw;
    yawLeftError = startYaw - 20;
    if(yawLeftError < -180)
      yawLeftError = Math.abs(yawLeftError)-((-yawLeftError-180)*2);
    yawRightError = startYaw + 20;
    if (yawRightError > 180)
      yawRightError = ((yawRightError-180)*2)-yawRightError;
    }
  }

public void brakeMode(boolean in){
  if (in){
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    brakeMode = true;
  }
  else{
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
    brakeMode = false;
  }
}

public void turn180(){
  
  turnCalc = turnPID.calculate(navx.getYaw(), endYaw);
  drive.tankDrive(-turnCalc, turnCalc);
}

public void turn180Init(){
  endYaw = navx.getYaw()+180;
  if (endYaw > 180)
      endYaw = ((endYaw-180)*2)-endYaw;
  yawLeftError = endYaw - 2;
  if(yawLeftError < -180)
    yawLeftError = Math.abs(yawLeftError)-((-yawLeftError-180)*2);
  yawRightError = endYaw + 2;
  if (yawRightError > 180)
    yawRightError = ((yawRightError-180)*2)-yawRightError;
  turnPID.setSetpoint(endYaw);
}
public boolean getTurnDone(){return turnPID.atSetpoint();}

public void toggleBrakeMode(){
  brakeMode(brakeMode == false);
}

public boolean autoDrive(double ft){
  boolean done = false;
  autoDrivePID.setSetpoint(-ft * Constants.encoderPositionPerFoot);
  double autoCalc = autoDrivePID.calculate(frontRight.getSelectedSensorPosition(),ft * Constants.encoderPositionPerFoot);
  drive.tankDrive(autoCalc,autoCalc);
  if(autoDrivePID.atSetpoint()){return false;}
  return true;
}

  public void drive(double leftValue, double rightValue){
    if (!BALANCING) {
      drive.tankDrive(leftValue, rightValue);
  } else {
      calculatedPower = balancePID.calculate(-navx.getPitch(), gyroSetpointAngle);
      drive.tankDrive(calculatedPower, calculatedPower);
      if (navx.getPitch() > -2 && navx.getPitch() < 2)
      {
        balanceTime+=0.025;
        if(balanceTime>=3){
          BALANCING = false;
          brakeMode(true);
        }
      }
      else{
        balanceTime = 0;
      }
  }
  }
  //private TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.kMaxSpeedMetersPerSecond, Constants.Drive.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Drive.kDriveKinematics).addConstraint(Constants.Drive.voltageConstraint);
  //private Pose2d startPose = new Pose2d();
  //public Trajectory startTrajectory =
  //        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(startPose, new ArrayList<Translation2d>(), new Pose2d(), config);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavX Pitch",navx.getPitch());
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putBoolean("Balancing", BALANCING);
    SmartDashboard.putBoolean("Brake Mode", brakeMode);
    SmartDashboard.putBoolean("At Setpoint: ", balancePID.atSetpoint());
    SmartDashboard.putNumber("Left Position", frontLeft.getSelectedSensorPosition()*-2);
    SmartDashboard.putNumber("Right Position", frontRight.getSelectedSensorPosition());
    getPoseFromOdometry();
    // This method will be called once per scheduler run
  }
}
