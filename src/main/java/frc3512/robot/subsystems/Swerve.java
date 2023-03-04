package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleArrayEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.sim.GyroSim;
import frc3512.lib.util.PhotonCameraWrapper;
import frc3512.robot.Constants;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro;
  private final GyroSim gyroSim;
  private final PhotonCameraWrapper camera;

  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] mSwerveMods;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private Field2d field;
  private final SpartanDoubleEntry gyroYaw;
  private final SpartanDoubleArrayEntry moduleIntegratedPositions;
  private final SpartanDoubleArrayEntry moduleAbsolutePositions;
  private final SpartanDoubleArrayEntry moduleDriveVelocities;
  private final SpartanDoubleArrayEntry moduleDrivePositions;
  private int counter = 0;


  /** Subsystem class for the swerve drive. */
  public Swerve() {
    camera = new PhotonCameraWrapper();
    gyro = new WPI_Pigeon2(Constants.SwerveConstants.pigeonID);
    gyroSim = new GyroSim(gyro);
    gyro.configFactoryDefault();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    Timer.delay(2.0);

    //New code that moves modules to angle offsets at before periodic
    /*SwerveModuleState SwerveModuleState0 = new SwerveModuleState(0, Constants.SwerveConstants.Mod0.constants.angleOffset);
    SwerveModuleState SwerveModuleState1 = new SwerveModuleState(0, Constants.SwerveConstants.Mod1.constants.angleOffset); 
    SwerveModuleState SwerveModuleState2 = new SwerveModuleState(0, Constants.SwerveConstants.Mod2.constants.angleOffset); 
    SwerveModuleState SwerveModuleState3 = new SwerveModuleState(0, Constants.SwerveConstants.Mod3.constants.angleOffset); 
    mSwerveMods[0].setCalibrateAngle(SwerveModuleState0);
    mSwerveMods[1].setCalibrateAngle(SwerveModuleState1);
    mSwerveMods[2].setCalibrateAngle(SwerveModuleState2);
    mSwerveMods[3].setCalibrateAngle(SwerveModuleState3);
    //Timer.delay(10);*/
    //end of new code

    resetModuleZeros();
    syncModuleEncoders();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics,
            getYaw(),
            getPositions(),
            new Pose2d());

    zeroGyro();
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    gyroYaw = new SpartanDoubleEntry("/Diagnostics/Swerve/Gyro/Yaw");
    moduleIntegratedPositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Integrated Positions");
    moduleAbsolutePositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Absolute Positions");
    moduleDriveVelocities =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Drive Velocity");
    moduleDrivePositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Drive Positions");

    //inverts left side Turn motors
    mSwerveMods[0].getTurnMotor().setInverted(true);
    mSwerveMods[2].getTurnMotor().setInverted(true);
    //inverts right side Turn Motors
    mSwerveMods[1].getTurnMotor().setInverted(true);
    mSwerveMods[3].getTurnMotor().setInverted(true);

    mSwerveMods[0].getDriveMotor().setInverted(false); 
    mSwerveMods[2].getDriveMotor().setInverted(false);

    mSwerveMods[1].getDriveMotor().setInverted(true);
    mSwerveMods[3].getDriveMotor().setInverted(true);
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(
                      translationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(
                      strafeSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(
                      rotationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));

          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(Constants.SwerveConstants.maxSpeed),
              rotationVal * Constants.SwerveConstants.maxAngularVelocity,
              true,
              true);
        })
        .withName("TeleopSwerve");
  }
  // Appears to be a pathfinder autonomous related command.
  public Command followTrajectory(PathPlannerTrajectory trajectory, boolean firstPath) {
    return Commands.sequence(
        run(
            () -> {
              if (firstPath) {
                this.resetOdometry(trajectory.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutonConstants.xControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.yControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.thetaControllerP, 0, 0),
            this::setModuleStates,
            this),
        this.drive(() -> 0.0, () -> 0.0, () -> 0.0));
  }
//Main drive function which will be called in TelopSwerve
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    if((translation.getY() <= 0.05 && translation.getX() <= 0.05 && rotation <= 0.05) && (translation.getY() >= -0.05 && translation.getX() >= -0.05 && rotation >= -0.05)) {
      syncModuleEncoders0();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetAbsolute(); 
      //Timer.delay(5);
   }
  }
  
  public void syncModuleEncoders() {
    for (SwerveModule mod : mSwerveMods) {
      mod.synchronizeEncoders();
      //Timer.delay(5);
    }
  }

  public void syncModuleEncoders0() {
      for (SwerveModule mod : mSwerveMods) {
        mod.synchronizeEncoders1();
        //Timer.delay(5);
    }
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroGyro() {
    // if (!RobotBase.isSimulation()) {
      gyro.setYaw(0);
    // } else {
    //   simIMU.setAngle(0);
    // }
    // swerveController.lastAngleScalar = 0;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void setYaw(double degrees) {
    gyro.setYaw(degrees);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
//periodic function updates all values
  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getPositions());

    Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(getPose());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - camPose.timestampSeconds);
    }

    moduleAbsolutePositions.set(
        new double[] {
          mSwerveMods[0].getAnglePosition(),
          mSwerveMods[1].getAnglePosition(),
          mSwerveMods[2].getAnglePosition(),
          mSwerveMods[3].getAnglePosition()
        });
    moduleIntegratedPositions.set(
        new double[] {
          mSwerveMods[0].getAnglePosition(),
          mSwerveMods[1].getAnglePosition(),
          mSwerveMods[2].getAnglePosition(),
          mSwerveMods[3].getAnglePosition()
        });
    moduleDriveVelocities.set(
        new double[] {
          mSwerveMods[0].getDriveVelocity(),
          mSwerveMods[1].getDriveVelocity(),
          mSwerveMods[2].getDriveVelocity(),
          mSwerveMods[3].getDriveVelocity()
        });
    moduleDrivePositions.set(
        new double[] {
          mSwerveMods[0].getDrivePosition(),
          mSwerveMods[1].getDrivePosition(),
          mSwerveMods[2].getDrivePosition(),
          mSwerveMods[3].getDrivePosition()
        });

    gyroYaw.set(getYaw().getDegrees());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Mod 0 CAN Coder", mSwerveMods[0].getAnglePosition());
    SmartDashboard.putNumber("Mod 1 CAN Coder", mSwerveMods[1].getAnglePosition());
    SmartDashboard.putNumber("Mod 2 CAN Coder", mSwerveMods[2].getAnglePosition());
    SmartDashboard.putNumber("Mod 3 CAN Coder", mSwerveMods[3].getAnglePosition());

    SmartDashboard.putNumber("Mod 0 Integrated Encoder", mSwerveMods[0].integratedAngleEncoder.getPosition());
    SmartDashboard.putNumber("Mod 1 Integrated Encoder", mSwerveMods[1].integratedAngleEncoder.getPosition());
    SmartDashboard.putNumber("Mod 2 Integrated Encoder", mSwerveMods[2].integratedAngleEncoder.getPosition());
    SmartDashboard.putNumber("Mod 3 Integrated Encoder", mSwerveMods[3].integratedAngleEncoder.getPosition());


    SmartDashboard.putBoolean("Mod 0 resetAbsolute", mSwerveMods[0].reseted);
    SmartDashboard.putBoolean("Mod 1 resetAbsolute", mSwerveMods[1].reseted);
    SmartDashboard.putBoolean("Mod 2 resetAbsolute", mSwerveMods[2].reseted);
    SmartDashboard.putBoolean("Mod 3 resetAbsolute", mSwerveMods[3].reseted);
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    ChassisSpeeds chassisSpeeds =
        Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
    gyroSim.setYaw(chassisSpeeds.omegaRadiansPerSecond);
  }
}
