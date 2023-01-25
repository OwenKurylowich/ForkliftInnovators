// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

/** Add your docs here. */
public class Vector2d {
    private double x;
    private double y;
    private double angle;
    private double magnitude;
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = calculateAngle();
        this.magnitude = calculateMagnitude();
    }
    private double calculateAngle() {
        if (x >= 0 && y >= 0) {
            return Math.atan(y/x);
        } else if (y > 0) {
            return Math.atan(y/x) + Math.PI / 2;
        } else {
            return Math.atan(y/x) + Math.PI;
        }
    }
    public double getAngle() {
        return this.angle;
    }
    private double calculateMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
    public double getMagnitude() {
        return this.magnitude;
    }}
