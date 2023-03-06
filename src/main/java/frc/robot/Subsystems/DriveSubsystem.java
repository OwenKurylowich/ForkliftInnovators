// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
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
import frc.robot.Commands.autonomousCommand;
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
    private final double bkP = 0.0365;   //0.0325 for no extrsa weight, 0.04 for extra weight, 0.045 with max weight
    private final double bkI = 0.00001;   //0.14 for no extra weight, 0.15 for extra weight,0.018 with max weight
    private final double bkD = 0.013;   // 0.011 for normal and extra wweight, 
    private final double gyroSetpointAngle = 0;
    private final PIDController balancePID;

    private final double tkP = 0.02;
    private final double tkI = 0.0001;
    private final double tkD = 0.1;
    private final PIDController turnPID;
    private double turnTime = 0;

    //private final double akP = 0.0000375;
    private final double akP = 0.00000175;
    private final double akI = 0.0;
    private final double akD = 0.000;//changed from 0.001 to 0.001//
    private final PIDController autoDriveRightPID;
    private final PIDController autoDriveLeftPID;
    private double autoCalcRight = 0;
    private double autoCalcLeft = 0;

    private final double adkP = 0.125;
    private final double adkI = 0.0;
    private final double adkD = 0.0;
    public PIDController autoDrivePID;

    private double calculatedPower = 0;
    private double turnCalc = 0;
    private double balanceTime = 0;
    private boolean brakeMode = false;
    private float startYaw = 0;
    private float endYaw = 0;
    private float yawLeftError = 0;
    private float yawRightError = 0;
	public Object toggleBrakeMode;
    //public Object toggleBarkeMode;
    public boolean firstDriveRun;
    public Object lineUp;
    public boolean atSetpoint;


    //intializes the autoDrivePID//
public void autoDrivePID(){;
}

  //Fixes 3 prolbems, makes another 1//
  /** Creates a new Drive. */
   public DriveSubsystem()
   {
    right.setInverted(true);
    right.setInverted(true);

    frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);

      frontLeft.configClosedloopRamp(Constants.encoderPositionPerFoot/1000);
      frontLeft.config_kP(0, akP);
      frontLeft.config_kI(0, akI);
      frontLeft.config_kD(0, akD);

      frontRight.configClosedloopRamp(Constants.encoderPositionPerFoot/1000);
      frontRight.config_kP(0, akP);
      frontRight.config_kI(0, akI);
      frontRight.config_kD(0, akD);

    navx.reset();
    navx.resetDisplacement();
    balancePID = new PIDController(bkP, bkI, bkD);
    balancePID.setSetpoint(gyroSetpointAngle);
    balancePID.setTolerance(1.5);
    balancePID.enableContinuousInput(-180, 180);
    turnPID = new PIDController(tkP, tkI, tkD);
    turnPID.setTolerance(2);
    turnPID.enableContinuousInput(-180, 180);
    autoDriveRightPID = new PIDController(akP, akI, akD);
    autoDriveRightPID.setTolerance(5000);
    autoDriveLeftPID = new PIDController(akP, akI, akD);
    autoDriveLeftPID.setTolerance(5000);
    //autoDrivePID.enableContinuousInput(-0.2*Constants.encoderPositionPerFoot, 0.2*Constants.encoderPositionPerFoot);
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

public AHRS getNavx(){return navx;}

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
}//canged boolean to void
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
  return (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2;
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
public void turnToPointInit(){
  turnTime = 0;
}
public boolean turnToPoint(float startYaw2){
  turnPID.setSetpoint(startYaw2);
  turnCalc = turnPID.calculate(navx.getYaw(), startYaw2);
  drive.tankDrive(-turnCalc, turnCalc);
  if(turnPID.atSetpoint()){
    turnTime+=0.025;
    if(turnTime >= 3)
      return false;
  }
  return true;
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
   //frontRight.set(ControlMode.Position, ft*Constants.encoderPositionPerFoot);
  //  backRight.follow(frontRight);
   //frontLeft.set(ControlMode.Position, ft*Constants.encoderPositionPerFoot);
  //  backLeft.follow(frontLeft);

  autoDriveRightPID.setSetpoint(ft * Constants.encoderPositionPerFoot);
  autoDriveLeftPID.setSetpoint(ft * Constants.encoderPositionPerFoot);
  autoCalcRight = autoDriveRightPID.calculate(frontRight.getSelectedSensorPosition(),ft * Constants.encoderPositionPerFoot);
  autoCalcLeft = autoDriveLeftPID.calculate((frontLeft.getSelectedSensorPosition()*-1),ft * Constants.encoderPositionPerFoot);
  drive.tankDrive(-autoCalcLeft,-autoCalcRight);
  if(autoDriveRightPID.atSetpoint() && autoDriveLeftPID.atSetpoint())
  {
      return true;
  }
  // frontRight.set(ControlMode.Position, ft*Constants.encoderPositionPerFoot);
  // backRight.follow(frontRight);
  // frontLeft.set(ControlMode.Position, ft*Constants.encoderPositionPerFoot);
  // backLeft.follow(frontLeft);


  return false;
}

public boolean autoBal(){
  BALANCING = true;
  calculatedPower = balancePID.calculate(-navx.getPitch(), gyroSetpointAngle);
    drive.tankDrive(calculatedPower, calculatedPower);
    if (navx.getPitch() > -1 && navx.getPitch() < 1){
      balanceTime+=0.02;
      if(balanceTime>=5){
        BALANCING = false;;
      }
    }
    else{
      balanceTime = 0;
    }
    return BALANCING;
}
public boolean autoOffBal(){
  BALANCING = true;
  calculatedPower = balancePID.calculate(navx.getPitch(), gyroSetpointAngle);
    drive.tankDrive(calculatedPower, calculatedPower);
    if (navx.getPitch() > -1 && navx.getPitch() < 1){
      balanceTime+=0.02;
      if(balanceTime>=5){
        BALANCING = false;;
      }
    }
    else{
      balanceTime = 0;
    }
    return BALANCING;
}
  public void drive(double leftValue, double rightValue){
    if (!BALANCING) {
     // drive.tankDrive(leftValue, leftValue);
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
//      edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(startPose, new ArrayList<Translation2d>(), new Pose2d(), config);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavX Pitch",navx.getPitch());
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putBoolean("Balancing", BALANCING);
    SmartDashboard.putBoolean("Brake Mode", brakeMode);
    SmartDashboard.putBoolean("At Setpoint: ", balancePID.atSetpoint());
    SmartDashboard.putNumber("Left Position", frontLeft.getSelectedSensorPosition() * -1);
    SmartDashboard.putNumber("Right Position", frontRight.getSelectedSensorPosition());
    getPoseFromOdometry();
    // This method will be called once per scheduler run
  }

  public static void setInstance(DriveSubsystem instance) {
    DriveSubsystem.instance = instance;
  }

  public static void setController(RamseteController controller) {
    DriveSubsystem.controller = controller;
  }

  public static void setRamsetePIDController(PIDController ramsetePIDController) {
    DriveSubsystem.ramsetePIDController = ramsetePIDController;
  }

  public void setNavx(AHRS navx) {
    this.navx = navx;
  }

  public void setNavxResetOffsetX(double navxResetOffsetX) {
    this.navxResetOffsetX = navxResetOffsetX;
  }

  public void setNavxResetOffsetY(double navxResetOffsetY) {
    this.navxResetOffsetY = navxResetOffsetY;
  }

  public void setNavxResetOffsetRot(double navxResetOffsetRot) {
    this.navxResetOffsetRot = navxResetOffsetRot;
  }

  public void setOdometry(DifferentialDriveOdometry odometry) {
    this.odometry = odometry;
  }

  public void setForkliftOdometry(ForkliftOdometry forkliftOdometry) {
    this.forkliftOdometry = forkliftOdometry;
  }

  public void setBALANCING(boolean bALANCING) {
    BALANCING = bALANCING;
  }

  public void setTurnTime(double turnTime) {
    this.turnTime = turnTime;
  }

  public void setAutoCalcRight(double autoCalcRight) {
    this.autoCalcRight = autoCalcRight;
  }

  public void setAutoCalcLeft(double autoCalcLeft) {
    this.autoCalcLeft = autoCalcLeft;
  }

  public void setCalculatedPower(double calculatedPower) {
    this.calculatedPower = calculatedPower;
  }

  public void setTurnCalc(double turnCalc) {
    this.turnCalc = turnCalc;
  }

  public void setBalanceTime(double balanceTime) {
    this.balanceTime = balanceTime;
  }

  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  public void setStartYaw(float startYaw) {
    this.startYaw = startYaw;
  }

  public void setEndYaw(float endYaw) {
    this.endYaw = endYaw;
  }

  public void setYawLeftError(float yawLeftError) {
    this.yawLeftError = yawLeftError;
  }

  public void setYawRightError(float yawRightError) {
    this.yawRightError = yawRightError;
  }

  public WPI_TalonSRX getFrontRight() {
    return frontRight;
  }

  public WPI_TalonSRX getBackRight() {
    return backRight;
  }

  public WPI_TalonSRX getFrontLeft() {
    return frontLeft;
  }

  public WPI_TalonSRX getBackLeft() {
    return backLeft;
  }

  public MotorControllerGroup getRight() {
    return right;
  }

  public MotorControllerGroup getLeft() {
    return left;
  }

  public DifferentialDrive getDrive() {
    return drive;
  }

  public static RamseteController getController() {
    return controller;
  }

  public static PIDController getRamsetePIDController() {
    return ramsetePIDController;
  }

  public double getNavxResetOffsetX() {
    return navxResetOffsetX;
  }

  public double getNavxResetOffsetY() {
    return navxResetOffsetY;
  }

  public double getNavxResetOffsetRot() {
    return navxResetOffsetRot;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public ForkliftOdometry getForkliftOdometry() {
    return forkliftOdometry;
  }

  public boolean isBALANCING() {
    return BALANCING;
  }

  public double getBkP() {
    return bkP;
  }

  public double getBkI() {
    return bkI;
  }

  public double getBkD() {
    return bkD;
  }

  public double getGyroSetpointAngle() {
    return gyroSetpointAngle;
  }

  public PIDController getBalancePID() {
    return balancePID;
  }

  public double getTkP() {
    return tkP;
  }

  public double getTkI() {
    return tkI;
  }

  public double getTkD() {
    return tkD;
  }

  public PIDController getTurnPID() {
    return turnPID;
  }

  public double getTurnTime() {
    return turnTime;
  }

  public double getAkP() {
    return akP;
  }

  public double getAkI() {
    return akI;
  }

  public double getAkD() {
    return akD;
  }

  public PIDController getAutoDriveRightPID() {
    return autoDriveRightPID;
  }

  public PIDController getAutoDriveLeftPID() {
    return autoDriveLeftPID;
  }

  public double getAutoCalcRight() {
    return autoCalcRight;
  }

  public double getAutoCalcLeft() {
    return autoCalcLeft;
  }

  public PIDController getAutoDrivePID() {
    return autoDrivePID;
  }

  public double getCalculatedPower() {
    return calculatedPower;
  }

  public double getTurnCalc() {
    return turnCalc;
  }

  public double getBalanceTime() {
    return balanceTime;
  }

  public boolean isBrakeMode() {
    return brakeMode;
  }

  public float getStartYaw() {
    return startYaw;
  }

  public float getEndYaw() {
    return endYaw;
  }

  public float getYawLeftError() {
    return yawLeftError;
  }

  public float getYawRightError() {
    return yawRightError;
  }

  public double getString() {
    return getString();
  }

  public void setAutoDrivePID(PIDController autoDrivePID) {
    this.autoDrivePID = autoDrivePID;
  }

public boolean turnToPoint(boolean lineUp) {
  return lineUp;
}

public void setBrakeModeOne(boolean b) {
}
}
