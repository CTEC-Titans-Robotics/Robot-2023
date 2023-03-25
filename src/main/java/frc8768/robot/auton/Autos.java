package frc8768.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc8768.robot.Constants;
import frc8768.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;

@SuppressWarnings("unused")
public final class Autos {

  private final Swerve swerve;
  private final SendableChooser<Command> autonChooser;
  private final HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autonBuilder;


  public Autos(Swerve swerve) {
    this.swerve = swerve;

    eventMap = new HashMap<>();
    setMarkers();

    autonBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            new PIDConstants(Constants.AutonConstants.xyControllerP, 0.0, 0.0),
            new PIDConstants(Constants.AutonConstants.thetaControllerP, 0.0, 0.0),
            swerve::setChassisSpeeds,
            eventMap,
            true,
            swerve);

    autonChooser = new SendableChooser<>();
    autonChooser.addOption("No-op", new InstantCommand());
    autonChooser.addOption("Left Side Platform", leftSidePlatform());
    autonChooser.addOption("Left Side Community", leftSideCommunity());
    autonChooser.addOption("Right Side Platform", rightSidePlatform());
    autonChooser.setDefaultOption("Right Side Community", rightSideCommunity());
    autonChooser.addOption("Center Platform", centerPlatform());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void setMarkers() {
    eventMap.put("Wait 1 second", new WaitCommand(1));
    // EventMap.put("Kick it!", )
    eventMap.put("lock drivetrain", new InstantCommand(() -> swerve.lock()));
    eventMap.put("halt drivetrain", new InstantCommand(() -> swerve.halt()));
  }
  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command leftSidePlatform() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Left Side Platform", Constants.AutonConstants.constraints));
  }

  public Command levelOut() {
    // return new InstantCommand(() -> {
    //   while(true) {
    //     Rotation2d rot = swerve.getGyroRot();
    //     if(rot.getDegrees() <= 0.4 || rot.getDegrees() >= -0.4) {
    //       break;
    //     }
    //     double dir;
    //     if(rot.getDegrees() > 0.4) {
    //       dir = -0.0254;
    //     } else {
    //       dir = 0.0254;
    //     }
    //     swerve.drive(new Translation2d(0, dir), 0, true, false);
    //   }
    // });

    return new BalanceChassisCommand(swerve);
  }

  public Command leftSideCommunity() {
    return autonBuilder.fullAuto(
            PathPlanner.loadPath("Left Side Community", Constants.AutonConstants.constraints));
  }

  public Command rightSidePlatform() {
    return autonBuilder.fullAuto(
            PathPlanner.loadPath("Right Side Platform", Constants.AutonConstants.constraints)).andThen(new BalanceChassisCommand(swerve));
  }

  public Command rightSideCommunity() {
    return autonBuilder.fullAuto(
            PathPlanner.loadPath("Right Side Community", Constants.AutonConstants.constraints));
  }

  public Command centerPlatform() {
    return autonBuilder.fullAuto(
            (PathPlannerTrajectory) PathPlanner.loadPath("Center Platform", Constants.AutonConstants.constraints).
            concatenate(PathPlanner.loadPath("Center Platform 2", Constants.AutonConstants.constraints))).andThen(new BalanceChassisCommand(swerve));
  }
  public Command otherCenterPlatformPath(){
    return autonBuilder.fullAuto(
            (PathPlannerTrajectory) PathPlanner.loadPath("Center Platform", Constants.AutonConstants.constraints)).andThen(new backBalanceChassisCommand(swerve));
  }
}
