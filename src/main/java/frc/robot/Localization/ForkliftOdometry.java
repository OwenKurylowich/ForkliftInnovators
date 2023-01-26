// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

/** Add your docs here. */
public class ForkliftOdometry {
    private Pose2d pose;
    private double trackWidth;
    private double longitudinalDist;
    private WPI_TalonSRX frontLeft;
    private WPI_TalonSRX frontRight;
    private double prevRightFront = 0;
    private double prevLeftFront = 0;
    private double prevRightBack = 0;
    private double prevLeftBack = 0;

    public enum Mode {
        USING_NAVX,
        NO_GYRO;
    }
    private Mode mode = Mode.NO_GYRO;
    public ForkliftOdometry(Pose2d pose, double trackWidth,
                            double longitudinalDist,
                            WPI_TalonSRX frontLeft,
                            WPI_TalonSRX frontRight
                            ) {
        this.pose = pose;
        this.trackWidth = trackWidth;
        this.longitudinalDist = longitudinalDist;
        this.frontLeft = frontLeft;
        this.frontLeft.setSelectedSensorPosition(0);
        this.frontRight = frontRight;
        this.frontRight.setSelectedSensorPosition(0);
    }
    public ForkliftOdometry(double trackWidth, double longitudinalDist, WPI_TalonSRX frontLeft, WPI_TalonSRX frontRight) {
        this(new Pose2d(), trackWidth, longitudinalDist, frontLeft, frontRight);
    }
    public void setMode(Mode mode) {
        this.mode = mode;
    }
    public Pose2d calculate() {
        double rfEncoder = frontLeft.getSelectedSensorPosition();
        double lfEncoder = frontRight.getSelectedSensorPosition();
        double deltaRightFrontEncoder = rfEncoder - prevRightFront;
        double deltaLeftFrontEncoder = lfEncoder - prevLeftFront;
        prevRightFront = rfEncoder;
        prevLeftFront = lfEncoder;
        double rightSide = deltaRightFrontEncoder;
        double leftSide = deltaLeftFrontEncoder;
        double dDriveHeading = (rightSide - leftSide) / trackWidth;
        double radius = rightSide / dDriveHeading - trackWidth / 2;
        double dxDrive = radius - radius * Math.cos(dDriveHeading);
        double dyDrive = radius + radius * Math.sin(dDriveHeading);
        double back = Constants.Drive.backDistancePerPulse / Constants.Drive.mecanumDistancePerPulse * (deltaLeftFrontEncoder + deltaRightFrontEncoder);
        double dTurnHeading = back / longitudinalDist;
        double dxTurn = longitudinalDist * Math.sin(dTurnHeading);
        double dyTurn = longitudinalDist - longitudinalDist * Math.cos(dTurnHeading);
        Vector2d displacement = new Vector2d(dxDrive + dxTurn, dyDrive + dyTurn);
        double dHeading = dDriveHeading + dTurnHeading;
        double totalHeading = pose.getRotation().getRadians() + dHeading;
        double vectorAngle = displacement.getAngle() + pose.getRotation().getRadians(); //gets the angle of the displacement vector relative to the field plane
        double x = pose.getX() + displacement.getMagnitude() * Math.cos(vectorAngle);
        double y = pose.getY() + displacement.getMagnitude() * Math.sin(vectorAngle);
        pose = new Pose2d(x, y, new Rotation2d(totalHeading));
        return pose;
    }
    public Pose2d getPose() {
        return this.pose;
    }}
