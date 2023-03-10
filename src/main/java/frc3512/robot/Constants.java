package frc3512.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/** Constants for the robot project */
public final class Constants {

  /* Field related constants */
  public static final class FieldConstants {
    // List of possible scoring locations as Pose2d objects
    public static final List<Pose2d> scoringPositions =
        List.of(
            new Pose2d(
                new Translation2d(0.555, 7.436),
                Rotation2d.fromRadians(Math.PI)), // Red loading double station
            new Pose2d(new Translation2d(0.555, 6.146), Rotation2d.fromRadians(Math.PI)),
            new Pose2d(
                new Translation2d(15.03, 5.061),
                Rotation2d.fromDegrees(0.0)), // Red node scoring locations
            new Pose2d(new Translation2d(15.03, 4.405), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 3.846), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 3.298), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 2.74), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 2.2), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 1.62), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 1.06), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 0.52), Rotation2d.fromDegrees(0.0)),
            new Pose2d(
                new Translation2d(15.64, 7.430),
                Rotation2d.fromDegrees(0.0)), // Blue loading double substation
            new Pose2d(new Translation2d(15.64, 6.16), Rotation2d.fromDegrees(0.0)),
            new Pose2d(
                new Translation2d(1.598, 4.996),
                Rotation2d.fromRadians(-Math.PI)), // Blue node scoring locations
            new Pose2d(new Translation2d(1.598, 4.373), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 3.85), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 3.3), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 2.75), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 2.2), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 1.63), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 1.05), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 0.5), Rotation2d.fromRadians(-Math.PI)));
  }

  /** General robot constants */
  public static final class GeneralConstants {
    // Enable or disable competition mode
    public static final boolean tuningMode = false;

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    public static final double voltageComp = 12.5;

    // Hold time on motor brakes when disabled
    public static final double wheelLockTime = 10;

    public static final double robotMass = (148 - 20.3) * 0.453592;
    public static final double chassisMass = robotMass;
    public static final Translation3d chassisCG = new Translation3d(0, 0, Units.inchesToMeters(8));
    public static final double loopTime = 0.13;
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

  /** Constants revolving around the elevator subsystem. */
  public static final class ElevatorConstants {
    public static final int leftMotorID = 17;
    public static final int rightMotorID = 18;

    public static final double maxVelocityMeterPerSecond = 1.75;
    public static final double maxAccelerationMeterPerSecondSquared = 0.75;

    public static final double pGain = 0.0;
    public static final double iGain = 0.0;
    public static final double dGain = 0.0;
  }

  /** Constants revolving around the arm subsystem. */
  public static final class ArmConstants {
    public static final double offset = 10; 
    public static final int leftMotorID = 19;
    public static final int rightMotorID = 20;

    public static final double pGain = 0.0;
    public static final double iGain = 0.0;
    public static final double dGain = 0.0;

    public static final double maxVelocityRadPerSecond = 3;
    public static final double maxAccelerationRadPerSecSquared = 10;

    public static final double highGoalPosition = 95 - offset;
    public static final double midGoalPosition =  70 - offset;
    public static final double lowGoalPosition = 45 - offset;
 
  }

  /** Constants revolving around the intake subsystem. */
  public static final class IntakeConstants {
    public static final int intakeMotorID = 21;

    public static final double intakeCurrentThreshold = 40.0;
  }

  /** Constants revolving around auton modes. */
  public static final class AutonConstants {

    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;

    public static final PathConstraints constraints =
        new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration);

    public static final double xyControllerP = 1.5;
    public static final double thetaControllerP = 1.5;
  }

  public static class IDs {
    // pneumatic hub
    public static final int PNEUMATIC_CAN_ID = 20;
    // claw solenoid ports
    public static final int CLAW_OPEN_PORT = 0;
    public static final int CLAW_CLOSE_PORT = 1;

    // arm extension motor controller id
    public static final int EXTENSION_CAN_ID = 4;
    public static final int EXTENSION_ENC_ID = 0;

    // TODO: correct
    // power distribution hub
    public static final int PDH_CAN_ID = 42;

    // arm
    public static final int kArmMainCANID = 14;
    public static final int kArmFollowerCANID = 15;
    public static final int kArmCancoderCANID = 7;
  }

  public static class Arm {
    // 2,5 deg error position
    // 5 deg/s velocity error
    // 6v max output
    public static final double MIN_POS = 5;
    public static final double MAX_POS = 95;

    public static final double ZEROING_CUR = 0.2;

    public static final double kP = 0.33617;
    public static final double kI = 0;
    public static final double kD = 0.094215;
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 40; // 60
    public static final double kMaxAcceleration = 50; // 75
    public static final double kPosTolerance = 5;
    public static final double kVelTolerance = 10;
    public static final double kMaxControlEffort = 8;
    public static final double kS = 0.13079;
    public static final double kV = 0.081283;
    public static final double kA = 0.002638;
    public static final double kG = 0.14759;
    public static final double kDegreesPerRotation = 360;
  }
}
