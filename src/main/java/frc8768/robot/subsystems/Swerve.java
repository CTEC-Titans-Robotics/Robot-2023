package frc8768.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc8768.lib.HeadingController;
import frc8768.lib.SnapController;
import frc8768.lib.geometry.Rotation2d254;
import frc8768.robot.Constants;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  public final SwerveDrive swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(20.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(20.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(20.0);

  private final double m_tortoiseSpeed;
  private final double m_tortoiseAngularVelocity;

  private final double m_hareSpeed;
  private final double m_hareAngularVelocity;

  private Rotation2d254 m_heading;
  private HeadingController m_headingController;

  private SnapController m_snapController;
  private boolean m_snapEnabled;

  /** Subsystem class for the swerve drive. */
  public Swerve() {
    try {
      swerve =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    this.m_hareSpeed = swerve.swerveController.config.maxSpeed;
    this.m_hareAngularVelocity = swerve.swerveController.config.maxAngularVelocity;

    this.m_tortoiseSpeed = .75;
    this.m_tortoiseAngularVelocity = 1;

    m_headingController = new HeadingController();
    m_headingController.setTargetHeading(0);

    m_snapController = new SnapController();
    m_snapController.setTargetHeading(180);
    m_snapEnabled = false;
  }

  double hypot;
  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(
                      translationSup.getAsDouble(), 0.1));

          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(
                      strafeSup.getAsDouble(), 0.1));

          double rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(
                      rotationSup.getAsDouble(), 0.1));
          
          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(swerve.swerveController.config.maxSpeed),
              (rotationVal * swerve.swerveController.config.maxAngularVelocity),
              true,
              false,
              false);
        })
        .withName("TeleopSwerve");
  }

  public void setSnapHeading(double heading){
    m_snapEnabled = true;
      m_snapController.setTargetHeading(heading);
  }

  public Rotation2d254 getSnapHeading(){
      return m_snapController.getTargetHeading();
  }

  public void disableSnap(){
    m_snapEnabled = false;
    m_snapController.setTargetHeading(180);
  }

  public void drive(
      Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop, boolean headingCorrection) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop, headingCorrection);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public Rotation2d getGyroRot() {
    return swerve.getRoll();
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public void lock() {
    swerve.lockPose();
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  @Override
  public void periodic() {
    swerve.updateOdometry();
    // this should compensate for course deviations in heading in autonomous with pid tuning
  }

  public void tortoiseMode() {
    swerve.swerveController.config.maxSpeed = m_tortoiseSpeed;
    swerve.swerveController.config.maxAngularVelocity = m_tortoiseAngularVelocity;
  }

  public void hareMode() {
    swerve.swerveController.config.maxSpeed = m_hareSpeed;
    swerve.swerveController.config.maxAngularVelocity = m_hareAngularVelocity;
  }

  public void halt() {
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }
}
