// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.lib.geometry;

/** Add your docs here. */
public class Rotation2d254 {
    
    protected double mCos;
    protected double mSin;

    public Rotation2d254() {
        this(1,0, false);
    }
    public Rotation2d254(double x, double y, boolean normalize) {
        if(normalize) {
            double magnitude = Math.hypot(x, y);
            if (magnitude > 1e-12) {
                mCos = x / magnitude;
                mSin = y / magnitude;
            } else {
                mCos = 1;
                mSin = 0;
            }
        } else {
            mCos = x;
            mSin = y;
        }
    }
    public Rotation2d254(final Rotation2d254 otherRotation) {
        mCos = otherRotation.mCos;
        mSin = otherRotation.mSin;
    }
    public Rotation2d254(double thetaDegrees) {
        mCos = Math.cos(Math.toRadians(thetaDegrees));
        mSin = Math.sin(Math.toRadians(thetaDegrees));
    }

    public static Rotation2d254 fromDegrees(double angle) {
        return new Rotation2d254(angle);
    }

    public double cos() {
        return mCos;
    }
    public double sin() {
        return mSin;
    }

    public double getRadians() {
        return Math.atan2(mSin, mCos);
    }
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    public Rotation2d254 rotateBy(final Rotation2d254 other) {
        return new Rotation2d254(mCos * other.mCos - mSin * other.mSin, mCos * other.mSin + mSin * other.mCos, true);
    }

    public Rotation2d254 rotateBy(final double other) {
        double othermCos = Math.cos(Math.toRadians(other));
        double othermSin = Math.sin(Math.toRadians(other));

        return new Rotation2d254(mCos * othermCos - mSin * othermSin, mCos * othermSin + mSin * othermCos, true);
    }
    
    public double distance(final Rotation2d254 other) {
        return inverse().rotateBy(other).getRadians();
    }

    public Rotation2d254 inverse() {
        return new Rotation2d254(mCos, -mSin, false);
    }

    public Translation2d254 toVector2d() {
        return new Translation2d254(mCos, mSin);
    }
}
