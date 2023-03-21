package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax m_intake;

    private boolean m_isHeld;
    private boolean m_isRunning;
    private Timer m_timer;

    public Intake() {
        m_intake = new CANSparkMax(35, MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        // m_intake.setSecondaryCurrentLimit(15);

        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // sticky faults?
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10); // current and temp
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000); // motor position
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000); // analog sensor
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 60000); // encoder
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 60000); // encoder
        m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 60000); // encoder
        m_intake.setControlFramePeriodMs(10);

        m_intake.burnFlash();

        m_isRunning = false;
        m_isHeld = false;

        m_timer = new Timer();
    }

    @Override
    public void periodic() {
        if(!m_isHeld && m_timer.hasElapsed(1) && m_intake.getOutputCurrent() >= 80) {
            m_isHeld = true;
            m_isRunning = false;
            m_intake.set(-0.1);
            m_timer.stop();
            m_timer.reset();
        }
    }

    

    public void in() {
        if(!m_isHeld && !m_isRunning) {
            m_intake.set(-0.75);
            m_timer.reset();
            m_timer.start();
            m_isRunning = true; // needed for timer
        }
    }
    public void out() {
        if (m_isHeld) {
            m_isHeld = false;
            m_intake.set(0.75);
        }
    }
    public void stopMovement() {
        m_intake.set(0);
        m_isRunning = false;
    }

    public void printVoltage() {
        SmartDashboard.putNumber("Intake Current", m_intake.getOutputCurrent());
    }
}
