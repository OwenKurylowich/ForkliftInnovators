// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

/** Add your docs here. */
public class ForkliftOdometry {
    private Pose2d pose;
    private double trackWidth;
    private double longitudinalDist;
    private Encoder leftFrontEncoder;
    private Encoder rightFrontEncoder;
    private Encoder leftBackEncoder;
    private Encoder rightBackEncoder;
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
                            Encoder leftFrontEncoder,
                            Encoder rightFrontEncoder,
                            Encoder leftBackEncoder,
                            Encoder rightBackEncoder
                            ) {
        this.pose = pose;
        this.trackWidth = trackWidth;
        this.longitudinalDist = longitudinalDist;
        this.leftFrontEncoder = leftFrontEncoder;
        this.leftFrontEncoder.reset();
        this.rightFrontEncoder = rightFrontEncoder;
        this.rightFrontEncoder.reset();
        this.leftBackEncoder = leftBackEncoder;
        this.leftBackEncoder.reset();
        this.rightBackEncoder = rightBackEncoder;
        this.rightBackEncoder.reset();
    }
    public ForkliftOdometry(double trackWidth, double longitudinalDist, Encoder leftFrontEncoder,
                            Encoder rightFrontEncoder, Encoder leftBackEncoder, Encoder rightBackEncoder) {
        this(new Pose2d(), trackWidth, longitudinalDist, leftFrontEncoder, rightFrontEncoder, leftBackEncoder, rightBackEncoder);
    }
    public void setMode(Mode mode) {
        this.mode = mode;
    }
    public Pose2d calculate() {
        double rfEncoder = rightFrontEncoder.getDistance();
        double lfEncoder = leftFrontEncoder.getDistance();
        double rbEncoder = rightBackEncoder.getDistance();
        double lbEncoder = leftBackEncoder.getDistance();
        double deltaRightFrontEncoder = rfEncoder - prevRightFront;
        double deltaLeftFrontEncoder = lfEncoder - prevLeftFront;
        double deltaRightBackEncoder = rbEncoder - prevRightBack;
        double deltaLeftBackEncoder = lbEncoder - prevLeftBack;
        prevRightFront = rfEncoder;
        prevLeftFront = lfEncoder;
        prevRightBack = rbEncoder;
        prevLeftBack = lbEncoder;
        double rightSide = deltaRightFrontEncoder + deltaRightBackEncoder;
        double leftSide = deltaLeftFrontEncoder + deltaLeftBackEncoder;
        double back = Constants.Drive.backDistancePerPulse / Constants.Drive.mecanumDistancePerPulse * (deltaLeftBackEncoder + deltaRightBackEncoder);
        double dDriveHeading = (rightSide - leftSide) / trackWidth;
        double radius = rightSide / dDriveHeading - trackWidth / 2;
        double dxDrive = radius - radius * Math.cos(dDriveHeading);
        double dyDrive = radius + radius * Math.sin(dDriveHeading);
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
