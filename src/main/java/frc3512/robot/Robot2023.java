package frc3512.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.ArmExtension;
import frc3512.robot.subsystems.Claw;
import frc3512.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.Debouncer;


public class Robot2023 {
  // Robot subsystems
  private Swerve swerve = new Swerve(true);

  private Arm arm = new Arm();
  public Claw claw = new Claw();
  public ArmExtension extension = new ArmExtension();

  // Driver Control
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Joysticks
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
  private final CommandXboxController appendage =
      new CommandXboxController(Constants.OperatorConstants.appendageControllerPort);

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  /** Used for defining button actions. */
  public void configureButtonBindings() {
    driver.b().debounce(0.1, Debouncer.DebounceType.kBoth).onTrue(new InstantCommand(swerve::zeroGyro));
    driver.x().debounce(0.1, Debouncer.DebounceType.kBoth).whileTrue(new RepeatCommand(new InstantCommand(swerve::lock)));
    driver.leftTrigger().debounce(0.1, Debouncer.DebounceType.kBoth).onTrue(new InstantCommand(swerve::tortoiseMode));
    driver.leftTrigger().debounce(0.1, Debouncer.DebounceType.kRising).onFalse(new InstantCommand(swerve::hareMode));
    appendage.rightBumper().onTrue(new InstantCommand(() -> claw.openClaw()));
    appendage.rightTrigger().onTrue(new InstantCommand(() -> claw.closeClaw()));
  }

  public void periodic() {
    if(appendage.getRightY() > 0.05) {
      extension.negativeMovement(extension.reachedMinSup);
    } else if(appendage.getRightY() < -0.05) {
      extension.positiveMovement(extension.reachedMaxSup);
    } else {
      extension.stopMovement();
    }

    if(appendage.getLeftY() > 0.05) {
      arm.simpleArmNegativeMovement(arm.reachedMinSup);
    } else if(appendage.getLeftY() < -0.05) {
      arm.simpleArmPositiveMovement(arm.reachedMaxSup);
    } else {
      arm.stopMovement();
    }
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
