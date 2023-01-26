// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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
    private final double kP = 0.0525;
    private final double kI = 0;
    private final double kD = 0.005;
    private final double gyroSetpointAngle = 0;
    private final PIDController balancePID;
    private double calculatedPower = 0;



  


  /** Creates a new Drive. */
  public DriveSubsystem() {

    right.setInverted(true);
    frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    navx.reset();
    navx.resetDisplacement();
    balancePID = new PIDController(kP, kI, kD);
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
}

  public void drive(double leftValue, double rightValue){
    if (!BALANCING) {
      drive.tankDrive(leftValue, rightValue);
  } else {
      calculatedPower = balancePID.calculate(-navx.getPitch(), gyroSetpointAngle);
      drive.tankDrive(calculatedPower, calculatedPower);
  }
  }

  private TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.kMaxSpeedMetersPerSecond, Constants.Drive.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Drive.kDriveKinematics).addConstraint(Constants.Drive.voltageConstraint);
  private Pose2d startPose = new Pose2d();
  //public Trajectory startTrajectory =
  //        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(startPose, new ArrayList<Translation2d>(), new Pose2d(), config);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", frontLeft.getSelectedSensorVelocity());
    getPoseFromOdometry();
    // This method will be called once per scheduler run
  }
}