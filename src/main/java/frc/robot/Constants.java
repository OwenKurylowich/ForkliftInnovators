// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/** Add your docs here. */
public class Constants {
    public static int frontRightCAN = 0;
    public static int backRightCAN = 1;
    public static int frontLeftCAN = 2;
    public static int backLeftCAN = 3;

    public static final double deadZoneDefault = 0.1;

    public static double encoderPositionPerFoot = 13500;

    public static class Drive {
        public static final double tractionWheelDistancePerPulse = 2; //I'm not sure it that is meters or cm
        public static final double mecanumDistancePerPulse = 0.7071 * tractionWheelDistancePerPulse;
        public static final double backDistancePerPulse = 1;
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.69;
        public static final double kLongitudinalDistance = 1;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(Constants.Drive.ksVolts, Constants.Drive.kvVoltSecondsPerMeter, Constants.Drive.kaVoltSecondsSquaredPerMeter);
        public static double maxVoltage = 10;
        public static final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, kDriveKinematics, maxVoltage);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final int leftFrontEncoderPorts[] = {0, 1};
        public static final int rightFrontEncoderPorts[] = {2, 3};
        public static final int leftBackEncoderPorts[] = {4, 5};
        public static final int rightBackEncoderPorts[] = {6, 7};
        public static final boolean leftFrontEncoderReversed = false;
        public static final boolean rightFrontEncoderReversed = false;
        public static final boolean leftBackEncoderReversed = false;
        public static final boolean rightBackEncoderReversed = false;
    }
}
