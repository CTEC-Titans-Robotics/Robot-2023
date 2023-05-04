package frc8768.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc8768.robot.auton.Autos;
import frc8768.robot.auton.BalanceChassisCommand;
import frc8768.robot.subsystems.*;
import edu.wpi.first.math.filter.Debouncer;


public class Robot2023 {
  // Robot subsystems

  Claw claw = new Claw();

  // Joysticks
  private final CommandXboxController m_commandDriverController =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
  private final CommandXboxController m_commandAppendageController =
      new CommandXboxController(Constants.OperatorConstants.appendageControllerPort);  

  /** Used for defining button actions. */
  public void configureButtonBindings() {
    
    m_commandAppendageController.rightTrigger(0.1).debounce(0.1, Debouncer.DebounceType.kBoth).onTrue(new InstantCommand(() -> {}));
   
  }

  public void periodic() {
    
  }
}

