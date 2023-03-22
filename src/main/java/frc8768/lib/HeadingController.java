// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc8768.lib.geometry.Rotation2d254;
import frc8768.lib.util.SynchronousPIDF;

/** Add your docs here. */
public class HeadingController {
    private Rotation2d254 targetHeading = Rotation2d254.fromDegrees(0);
    private double lastTimestamp;
    private double maxSmallError;

    private SynchronousPIDF pidControllerLargeError, pidControllerSmallError;
    public HeadingController() {
        // pidControllerLargeError = new SynchronousPIDF(0.00041, 0.0, 0.003, 0.0);
        // pidControllerSmallError = new SynchronousPIDF(0.0003, 0.0, 0.005, 0.0);
        pidControllerLargeError = new SynchronousPIDF(0.00041 * .0001, 0.0, 0.003 * .0001, 0.0);
        // pidControllerLargeError.setOutputRange(-.1, .1);
        // pidControllerSmallError.setOutputRange(-.1, .1);
        pidControllerSmallError = new SynchronousPIDF(0.0003 * .0001, 0.0, 0.005 * .0001, 0.0);

        maxSmallError = 20;
    }

    public void setTargetHeading(double heading) {
        targetHeading = Rotation2d254.fromDegrees(heading);
    }

    public double updateRotationCorrection(Rotation2d254 heading, double timestamp) {
        double error = targetHeading.rotateBy(heading.inverse()).getDegrees();
        double dt = timestamp - lastTimestamp;
        double output, smallError, largeError;

        smallError = pidControllerSmallError.calculate(-error, dt);
        largeError = pidControllerLargeError.calculate(-error, dt);

        if (Math.abs(error) > maxSmallError){
            output = largeError;
        }
        else{
            output = smallError;
        }

        SmartDashboard.putNumber("Target Heading", targetHeading.getDegrees());
        SmartDashboard.putNumber("Heading Contorller Error", error);
        return output;

    }
    

}
