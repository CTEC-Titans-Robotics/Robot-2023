package frc3512.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
//import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc3512.lib.sim.MotorSim;
import frc3512.lib.util.CANCoderUtil;
import frc3512.lib.util.CANCoderUtil.CANCoderUsage;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.SwerveModuleConstants;
import frc3512.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private double angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveConstants.driveKS,
          Constants.SwerveConstants.driveKV,
          Constants.SwerveConstants.driveKA);

  private final MotorSim driveMotorSim = new MotorSim();
  private final MotorSim angleMotorSim = new MotorSim();

  public boolean reseted = false;
  /**
   * Creates a new swerve module with NEO motors and a CTRE CANCoder.
   *
   * @param moduleNumber - Number of the module (0-3)
   * @param moduleIO - Module IO for the appropriate module
   */

  //Class takes a intetger for use in an array and the modules constants define the can id's of all module devices
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset.getDegrees();

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public CANSparkMax getTurnMotor() {
    return angleMotor;
  }
  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  private void configAngleEncoder() {
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.sensorCoefficient = 0.087890625;

    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(config);
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CANCoderUsage.kMinimal);

    angleEncoder.setPositionToAbsolute();
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(
        Constants.SwerveConstants.angleConversionFactor); // double check this
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);
    angleController.setPositionPIDWrappingMinInput(-180.0);
    angleController.setPositionPIDWrappingMaxInput(180.0);
    angleController.setPositionPIDWrappingEnabled(true);
    angleMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    angleMotor.setIdleMode(IdleMode.kCoast);
    angleMotor.burnFlash();
    integratedAngleEncoder.setPosition(0.0);
  }

  public void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
    driveMotor.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);
    driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(
        Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(
        Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.SwerveConstants.angleKP);
    driveController.setI(Constants.SwerveConstants.angleKI);
    driveController.setD(Constants.SwerveConstants.angleKD);
    driveController.setFF(Constants.SwerveConstants.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public void resetAbsolute() {
    //angleEncoder.configFactoryDefault();
    double absolutePosition = getAnglePosition() - angleOffset;

//checks if the current position is within a tollerance of the angle offset  
    /*double tollerance = 2;
     if ((angleOffset - tollerance) < absolutePosition && absolutePosition < (angleOffset + 0.5)){
      angleEncoder.setPosition(absolutePosition);
      reseted = true;
    } else {
      angleEncoder.setPosition(angleOffset);
    }  */
   angleEncoder.setPosition(absolutePosition);
   reseted = true;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  void setCalibrateAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;

    // if (RobotBase.isSimulation()) {
    //   driveMotorSim.updateSimVelocity(desiredState);
    //   angleMotorSim.setCurrentAngle(angle.getDegrees());
    // }
  }

  // public Rotation2d getCanCoder() {
  //   return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  // }

  public double getAnglePosition() {
    // return angleEncoder.getAbsolutePosition();

    // copied from how many layers of copying by Ryan
    boolean readingError = false;
    MagnetFieldStrength strength = angleEncoder.getMagnetFieldStrength();

    if (strength != MagnetFieldStrength.Good_GreenLED) {
      DriverStation.reportWarning(
          "CANCoder " + angleEncoder.getDeviceID() + " magnetic field is less than ideal.\n", false);
    }
    if (strength == MagnetFieldStrength.Invalid_Unknown
        || strength == MagnetFieldStrength.BadRange_RedLED) {
      readingError = true;
      DriverStation.reportWarning(
          "CANCoder " + angleEncoder.getDeviceID() + " reading was faulty.\n", false);
      return 0;
    }
    double angle = angleEncoder.getAbsolutePosition();

    // Taken from democat's library.
    // Source:
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    ErrorCode code = angleEncoder.getLastError();
    int ATTEMPTS = 3;
    for (int i = 0; i < ATTEMPTS; i++) {
      if (code == ErrorCode.OK) {
        break;
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
      }
      angle = angleEncoder.getAbsolutePosition();
      code = angleEncoder.getLastError();
    }
    if (code != ErrorCode.OK) {
      readingError = true;
      DriverStation.reportWarning(
          "CANCoder " + angleEncoder.getDeviceID() + " reading was faulty, ignoring.\n", false);
    }

    return angle;
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAnglePosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAnglePosition()));
  }

  public void synchronizeEncoders() {
    if (angleEncoder != null) {
      integratedAngleEncoder.setPosition(getAnglePosition() - angleOffset);
    }
  }

}
