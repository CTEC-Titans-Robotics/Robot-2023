// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.lib.geometry;


/** Add your docs here. */
public class Translation2d254 {

    protected double mX;
    protected double mY;
    public Translation2d254() {
        this(0,0);
    }
    public Translation2d254(double x, double y) {
        mX = x;
        mY = y;
    }
    public Translation2d254(final Translation2d254 otherVec) {
        mX = otherVec.mX;
        mY = otherVec.mY;
    }

    public static Translation2d254 fromPolar(Rotation2d254 direction, double magnitude) {
        return new Translation2d254(direction.mCos * magnitude, direction.mSin * magnitude);
    }

    public double norm() {
        return Math.hypot(mX, mY);
    }

    public double x() {
        return mX;
    }
    public double y() {
        return mY;
    }

    public Translation2d254 translateBy(final Translation2d254 other) {
        return new Translation2d254(mX + other.mX, mY + other.mY);
    }
    
    public Translation2d254 rotateBy(final Rotation2d254 rotation) {
        return new Translation2d254(mX * rotation.cos() - mY * rotation.sin(), mX * rotation.sin() + mY * rotation.cos());
    }

    public Rotation2d254 direction() {
        return new Rotation2d254(mX, mY, true);
    }

    public Translation2d254 inverse() {
        return new Translation2d254(-mX, -mY);
    }
    
    public Translation2d254 scale(double scaleFactor) {
        return new Translation2d254(mX * scaleFactor, mY * scaleFactor);
    }

    public double distance(final Translation2d254 other) {
        return this.translateBy(other.inverse()).norm();
    }
}
