package frc3512.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Robot;
import frc3512.robot.subsystems.Swerve;

public class BalanceChassisCommand extends CommandBase {
	private Swerve swerve;
	private PIDController pid;

	public BalanceChassisCommand(Swerve swerve) {
		this.pid = new PIDController(BalanceChassisConstants.kP, BalanceChassisConstants.kI, BalanceChassisConstants.kD);
		this.pid.setSetpoint(BalanceChassisConstants.kGroundAngle);
		this.pid.setTolerance(BalanceChassisConstants.kTolerance);

		this.swerve = swerve;
		this.addRequirements(this.swerve);
	}

	@Override
	public void execute() {
		double vxMeters = MathUtil.clamp(this.pid.calculate(this.swerve.getGyroRot().getDegrees()),
				-BalanceChassisConstants.kDriveSpeedMPS, BalanceChassisConstants.kDriveSpeedMPS);

		this.swerve.drive(new Translation2d(-vxMeters, 0), 0, true, false);
	}

	@Override
	public boolean isFinished() {
		return this.pid.atSetpoint();
	}
}