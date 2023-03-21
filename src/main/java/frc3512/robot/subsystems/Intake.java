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
    private boolean m_isIntaking;
    private boolean m_isExhausting;
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

        m_isIntaking = false;
        m_isHeld = false;
        m_isExhausting = false;

        m_timer = new Timer();
    }

    @Override
    public void periodic() {
        if(m_isIntaking && m_timer.hasElapsed(.25) && m_intake.getOutputCurrent() >= 80) {
            m_isHeld = true;
            m_isIntaking = false;
            m_intake.set(-0.1);
            m_timer.stop();
            m_timer.reset();
        }
        if (m_isIntaking && m_timer.hasElapsed(7.5)) {
            m_isIntaking = false;
            m_isHeld = false;
            m_intake.set(0);
            m_timer.stop();
            m_timer.reset();
        }
        if (m_isExhausting && m_timer.hasElapsed(.5)) {
            m_intake.set(0);
            m_isExhausting = false;
            m_timer.stop();
        }
    }

    

    public void drive() {
        if(!m_isHeld && !m_isIntaking) {
            m_intake.set(-0.75);
            m_timer.reset();
            m_timer.start();
            m_isIntaking = true; // needed for timer
        }
        if (m_isHeld) {
            m_isHeld = false;
            m_isExhausting = true;
            m_timer.reset();
            m_timer.start();
            m_intake.set(0.75);
        }
    }

    // public void printVoltage() {
    // //     SmartDashboard.putNumber("Intake Current", m_intake.getOutputCurrent());
    // // }
}
