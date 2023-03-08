package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final double m_hareSpeed;
  private final double m_hareAngularVelocity;

  private final double m_tortoiseSpeed;
  private final double m_tortoiseAngularVelocity;

  private final Timer m_timer = new Timer();
  private final boolean m_headingCorrection;
  private double m_lastTime = 0;
  private double m_angle = 0;

  /** Subsystem class for the swerve drive. */
  public Swerve(boolean p_headingCorrection) {
    try {
      swerve =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    this.m_hareSpeed = swerve.swerveController.config.maxSpeed;
    this.m_hareAngularVelocity = swerve.swerveController.config.maxAngularVelocity;

    this.m_tortoiseSpeed = 1;
    this.m_tortoiseAngularVelocity = 1;

    this.m_headingCorrection = p_headingCorrection;
    if (this.m_headingCorrection) {
      this.m_lastTime = this.m_timer.get();
    }

    swerve.swerveController.addSlewRateLimiters(translationLimiter, strafeLimiter, rotationLimiter);
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          double translationVal =
                  MathUtil.applyDeadband(
                      translationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband);
          double strafeVal =
                  MathUtil.applyDeadband(
                      strafeSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband);
          // deadband is applied twice, second time in library
          // double rotationVal =
          //     rotationLimiter.calculate(
          //         MathUtil.applyDeadband(
          //             rotationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          translationVal = Math.pow(translationVal, 3);
          strafeVal = Math.pow(strafeVal, 3);
          double rotationVal = Math.pow(rotationSup.getAsDouble(), 3);
          if (this.m_headingCorrection) {
            // Estimate the desired angle in radians.
            this.m_angle +=
                (rotationVal * (this.m_timer.get() - this.m_lastTime))
                    * swerve.swerveController.config.maxAngularVelocity;
            // Get the desired ChassisSpeeds given the desired angle and current angle.
            ChassisSpeeds correctedChassisSpeeds =
                swerve.swerveController.getTargetSpeeds(
                    strafeVal, translationVal, rotationVal, swerve.getYaw().getRadians());
            // Drive using given data points.
            swerve.drive(
                SwerveController.getTranslation2d(correctedChassisSpeeds),
                correctedChassisSpeeds.omegaRadiansPerSecond,
                true,
                false);
            this.m_lastTime = this.m_timer.get();
          } else {
            drive(
                new Translation2d(translationVal, strafeVal)
                    .times(swerve.swerveController.config.maxSpeed),
                rotationVal * swerve.swerveController.config.maxAngularVelocity,
                true,
                false);
          }
        })
        .withName("TeleopSwerve");
  }

  public void drive(
      Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
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
  }

  public void tortoiseMode() {
    swerve.swerveController.config.maxSpeed = m_tortoiseSpeed;
    swerve.swerveController.config.maxAngularVelocity = m_tortoiseAngularVelocity;
  }

  public void hareMode() {
    swerve.swerveController.config.maxSpeed = m_hareSpeed;
    swerve.swerveController.config.maxAngularVelocity = m_hareAngularVelocity;
  }
}
