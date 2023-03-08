package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmNew;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class Robot2023 {
  // Robot subsystems
  private Swerve swerve = new Swerve(true);

  private Arm m_arm = new Arm();
  public Claw claw = new Claw();
  public ArmExtension m_armExtension = new ArmExtension();

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
    m_driverController
        .b()
        .debounce(0.1, Debouncer.DebounceType.kBoth)
        .onTrue(new InstantCommand(swerve::zeroGyro));
    m_driverController
        .x()
        .debounce(0.1, Debouncer.DebounceType.kBoth)
        .whileTrue(new RepeatCommand(new InstantCommand(swerve::lock)));
    m_driverController
        .leftTrigger()
        .debounce(0.1, Debouncer.DebounceType.kBoth)
        .onTrue(new InstantCommand(swerve::tortoiseMode));
    m_driverController
        .leftTrigger()
        .debounce(0.1, Debouncer.DebounceType.kRising)
        .onFalse(new InstantCommand(swerve::hareMode));
    m_appendageController.rightBumper().onTrue(new InstantCommand(() -> claw.openClaw()));
    m_appendageController.rightTrigger().onTrue(new InstantCommand(() -> claw.closeClaw()));

    // commands for new implementations of appendages
    // m_appendageController
    //   .a()
    //       .onTrue(
    //           Commands.runOnce(
    //               () -> {
    //                 m_armExtension.setGoal(Constants.Extension.MIN_POS);
    //                 m_armExtension.enable();
    //               },
    //               m_armExtension));
    // m_appendageController
    //   .b()
    //       .onTrue(
    //           Commands.runOnce(
    //               () -> {
    //                 m_armExtension.setGoal(Constants.Extension.MAX_POS);
    //                 m_armExtension.enable();
    //               },
    //               m_armExtension));

    // m_appendageController
    //   .x()
    //       .onTrue(
    //           Commands.runOnce(
    //               () -> {
    //                 m_arm.setGoal(Constants.Arm.MIN_POS);
    //                 m_arm.enable();
    //               },
    //               m_arm));
    // m_appendageController
    //   .y()
    //       .onTrue(
    //           Commands.runOnce(
    //               () -> {
    //                 m_arm.setGoal(Constants.Arm.MAX_POS);
    //                 m_arm.enable();
    //               },
    //               m_arm));
                  
  }

  public void periodic() {
    if (m_appendageController.getRightY() > 0.05) {
      m_armExtension.negativeMovement(m_armExtension.reachedMinSup);
    } else if (m_appendageController.getRightY() < -0.05) {
      m_armExtension.positiveMovement(m_armExtension.reachedMaxSup);
    } else {
      m_armExtension.stopMovement();
    }

    if (m_appendageController.getLeftY() > 0.05) {
      m_arm.simpleArmNegativeMovement(m_arm.reachedMinSup);
    } else if (m_appendageController.getLeftY() < -0.05) {
      m_arm.simpleArmPositiveMovement(m_arm.reachedMaxSup);
    } else {
      m_arm.stopMovement();
    }
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> m_driverController.getRawAxis(translationAxis),
            () -> m_driverController.getRawAxis(strafeAxis),
            () -> m_driverController.getRawAxis(rotationAxis)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
