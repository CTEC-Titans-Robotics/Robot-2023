package frc3512.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants.Arm;
import edu.wpi.first.wpilibj.XboxController;
import frc3512.robot.subsystems.ArmOld;
// import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.ArmExtension;
import frc3512.robot.subsystems.ArmNew;
//import frc3512.robot.subsystems.ArmNewSub;
import frc3512.robot.auton.Autos;
import frc3512.robot.subsystems.ArmNew;
import frc3512.robot.subsystems.Claw;
import frc3512.robot.subsystems.Swerve;

import edu.wpi.first.math.filter.Debouncer;


public class Robot2023 {
  // Robot subsystems
  private Swerve swerve = new Swerve();

   private ArmOld m_armo = new ArmOld();
  // private ArmNew m_arm = new ArmNew();

  public Claw claw = new Claw();
  public Autos autos = new Autos(swerve);
  public ArmExtension extension = new ArmExtension();

  // Driver Control
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Joysticks
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
  private final CommandXboxController m_appendageController =
      new CommandXboxController(Constants.OperatorConstants.appendageControllerPort);
  

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  /** Used for defining button actions. */
  public void configureButtonBindings() {
    m_driverController.b().debounce(0.25, Debouncer.DebounceType.kBoth).onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    m_driverController.leftTrigger().whileTrue(new InstantCommand(() -> swerve.swerve.swerveController.config.maxSpeed = 1));
    m_driverController.leftTrigger().whileFalse(new InstantCommand(() -> swerve.swerve.swerveController.config.maxSpeed = 14.5));

    m_driverController
      .leftTrigger()
      .debounce(0.1, Debouncer.DebounceType.kBoth)
      .onTrue(new InstantCommand(swerve::tortoiseMode));
    m_driverController
    .leftTrigger()
    .debounce(0.1, Debouncer.DebounceType.kRising)
    .onFalse(new InstantCommand(swerve::hareMode));
    // m_driverController
    // .x()
    // .debounce(0.1, Debouncer.DebounceType.kBoth)
    // .onTrue(new InstantCommand(swerve::lock));

    m_appendageController.rightBumper().onTrue(new InstantCommand(() -> claw.openClaw()));
    m_appendageController.rightTrigger().onTrue(new InstantCommand(() -> claw.closeClaw()));
    m_appendageController.a().onTrue(new InstantCommand(() -> extension.zeroingProtocol()));

    // m_appendageController
    //   .a()
    //   .debounce(0.1, Debouncer.DebounceType.kBoth)
    //   .onTrue(m_arm.setLowCommand());
      
    // m_appendageController
    // .x()
    // .debounce(0.1, Debouncer.DebounceType.kBoth)
    // .onTrue(m_arm.setMidCommand());

    // m_appendageController
    //   .y()
    //   .debounce(0.1, Debouncer.DebounceType.kBoth)
    //   .onTrue(m_arm.setHighCommand());
  }
    

  public void periodic(

  ) {
    // if(appendageXbox.getAButton()) {
    //   m_arm.setLow();
    // }
    // if(appendageXbox.getXButton()) {
    //   m_arm.setMid();
    // }
    // if(appendageXbox.getYButton()) {
    //   m_arm.setHigh();
    // }
    if(m_appendageController.getRightY() > 0.05) {
      extension.negativeMovement(extension.reachedMinSup);
    } else if(m_appendageController.getRightY() < -0.05) {
      extension.positiveMovement(extension.reachedMaxSup);
    } else {
      extension.stopMovement();
    }
    //SmartDashboard.putNumber("Gearbox Throughbore", m_arm.m_rotationEncoder.get());

    if(m_appendageController.getLeftY() > 0.05) {
      m_armo.simpleArmNegativeMovement(m_armo.reachedMinSup);
    } else if(m_appendageController.getLeftY() < -0.05) {
      m_armo.simpleArmPositiveMovement(m_armo.reachedMaxSup);
    } else {
      m_armo.stopMovement();
    }
  }

  /* Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> m_driverController.getRawAxis(translationAxis),
            () -> m_driverController.getRawAxis(strafeAxis),
            () -> m_driverController.getRawAxis(rotationAxis)));
  }

  public void armTest() {
    extension.zeroingProtocol();
    //m_arm.zeroingProtocol();
  }
  public void armPrint (){
    // SmartDashboard.putNumber("Gearbox Throughbore", m_arm.getDistance());
    // m_arm.setLow();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getSelected();
  }

  public void testPeriodic() {
    SmartDashboard.putNumber("Gyro Roll", swerve.getGyroRot().getDegrees());
  }
}
