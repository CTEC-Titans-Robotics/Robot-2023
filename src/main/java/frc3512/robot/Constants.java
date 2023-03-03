package frc3512.robot;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc3512.lib.util.SwerveModuleConstants;

/** Constants for the robot project */
public final class Constants {

  /** General robot constants */
  public static final class GeneralConstants {
    // Enable or disable competition mode
    public static final boolean tuningMode = false; //leave false, never true, never change everrrrrr 

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    // Value to voltage compensate the motors for
    public static final double voltageComp = 12.0;
  }

  /** Constants revolving around joysticks */
  public static class OperatorConstants {
    // Driver controller port
    public static final int driverControllerPort = 0;

    // Appendage controller port
    public static final int appendageControllerPort = 1;
  }

  /** Constants revolving around the vision subsystem. */
  public static final class VisionConstants {
    // Camera name
    public static final String cameraName = "OV5647";

    // Robot to camera transform
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
            new Rotation3d(0.0, 0.0, 0.0));
  }

  /** Constants revolving around auton modes. */
  public static final class AutonConstants {
    public static final double maxVelocity = 1.0;
    public static final double maxAcceleration = 4.0;

    public static final PathConstraints constraints =
        new PathConstraints(maxVelocity, maxAcceleration);

    public static final double xControllerP = 1.0;
    public static final double yControllerP = 1.0;
    public static final double thetaControllerP = 1.0;
  }

  /** Constants revolving around the swerve subsystem */
  public static final class SwerveConstants {
    /* Gyro Constants */
    public static final int pigeonID = 21;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150/7) / 1.0); // 150/7:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int driveContinuousCurrentLimit = 35;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.002;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 1.75;
    public static final double driveKV = 4.0;
    public static final double driveKA = 6.0;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 0.5; // meters per second
    public static final double maxAngularVelocity = 0.25;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(87.71484375); 
      // Proto: -112.5 | bevel gears in 87.71484375 | bevel gears to the right 87.71484375
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1; // Proto: 2
      public static final int angleMotorID = 3; // Proto: 1
      public static final int canCoderID = 9; // Proto: 3
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1.845703125); 
      // Proto: -125.59569549560548 | bevel gears in 1.845703125 | bevel gears to the right -177.5390625
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 17; // Proto: 14
      public static final int angleMotorID = 19; // Proto: 13
      public static final int canCoderID = 6; // Proto : 2
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.435546875); 
      // Proto: 106.96289825439453 |  bevel gears in 60.29296875 | bevel gears to the right 61.435546875
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 10; // Proto: 4
      public static final int angleMotorID = 2; // Proto: 3
      public static final int canCoderID = 8; // Proto: 1
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(45.087890625); 
      // Proto: 149.677734375 | bevel gears in 45.087890625 | bevel gears to the right -135.439453125
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
}
