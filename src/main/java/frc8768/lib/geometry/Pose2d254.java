// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.lib.geometry;

/** Add your docs here. */
public class Pose2d254 {

    protected Translation2d254 mTranslation;
    protected Rotation2d254 mRotation;
    public Pose2d254() {
        mTranslation = new Translation2d254();
        mRotation = new Rotation2d254();
    }
    public Pose2d254(Translation2d254 translation, Rotation2d254 rotation) {
        mTranslation = translation;
        mRotation = rotation;
    }

    public static Pose2d254 fromTranslation(final Translation2d254 translation) {
        return new Pose2d254(translation, new Rotation2d254());
    }
    public static Pose2d254 fromRotation(final Rotation2d254 rotation) {
        return new Pose2d254(new Translation2d254(), rotation);
    }
    


    public Rotation2d254 getRotation() {
        return this.mRotation;
    }

    public Translation2d254 getTranslation() {
        return this.mTranslation;
    }

}
