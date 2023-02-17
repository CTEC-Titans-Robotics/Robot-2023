package frc3512.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.auton.Autos;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Claw;
import frc3512.robot.subsystems.Swerve;

public class Robot2023 {
  // Robot subsystems
  private Swerve m_swerve = new Swerve();

  // Autons
  private final Autos autos = new Autos(m_swerve);

  // Driver Control
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private Arm arm = new Arm();
  private Claw claw = new Claw();

  // Joysticks
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
  private final CommandXboxController appendage =
      new CommandXboxController(Constants.OperatorConstants.appendageControllerPort);

  /** Used for defining button actions. */
  public void configureButtonBindings() {

    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    appendage.b().onTrue(new InstantCommand(() -> claw.toggleClaw()));
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));
    arm.setDefaultCommand(arm.move(
            appendage::getLeftY,
            null,
            null
    ));
  }

  public Arm getArm() {
    return arm;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
