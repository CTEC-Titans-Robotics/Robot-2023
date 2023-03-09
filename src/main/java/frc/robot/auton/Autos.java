package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
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

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());
    autonChooser.addOption("Score 1, Balance", balanceMidZone());
    autonChooser.addOption("Score 2 Money Zone", score2MoneyZone());
    autonChooser.addOption("Score 3 Money Zone", score3MoneyZone());
    autonChooser.addOption("Score 2 Far Zone", score2FarZone());
    autonChooser.addOption("Score 3 Far Zone", score3FarZone());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command balanceMidZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Balance Mid Zone", Constants.AutonConstants.constraints));
  }

  public Command score2MoneyZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 2 Money Zone", Constants.AutonConstants.constraints));
  }

  public Command score3MoneyZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 3 Money Zone", Constants.AutonConstants.constraints));
  }

  public Command score2FarZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup("Score 2 Far Zone", Constants.AutonConstants.constraints));
  }

  public Command score3FarZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup("Score 3 Far Zone", Constants.AutonConstants.constraints));
  }
}
