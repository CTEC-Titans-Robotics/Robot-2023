// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc8768.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

/** Add your docs here. */
public class ArmMad extends SubsystemBase {

    private final CANCoder canCoder  = new CANCoder(20);

    private final CANSparkMax leader = new CANSparkMax(Constants.IDs.kArmFollowerCANID, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(Constants.IDs.kArmMainCANID, MotorType.kBrushless);
    private SparkMaxPIDController PIDcontroller;
    private RelativeEncoder encoder = leader.getAlternateEncoder(Type.kQuadrature, 8192);

    private DigitalInput limitSwitch = new DigitalInput(1);
    private boolean isZeroed = false;
    
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    

    public final double degreesToClicksFactor = 8192/360;
    public final double clicksToDegreesFactor = 360/8192;
    
    public ArmMad(){
        SparkMaxPIDController PIDcontroller = leader.getPIDController();
        PIDcontroller.setP(Constants.Arm.kP);
        PIDcontroller.setD(Constants.Arm.kD);   
        PIDcontroller.setI(Constants.Arm.kI); 
        follower.follow(leader);
        encoder.setPositionConversionFactor(8192);
        PIDcontroller.setPositionPIDWrappingMaxInput(Constants.Arm.MAX_POS);
        PIDcontroller.setPositionPIDWrappingMinInput(Constants.Arm.MIN_POS);
        PIDcontroller.setFeedbackDevice(encoder);
        
        //CANcoder config

        CANCoderConfiguration armconfig = new CANCoderConfiguration();
        armconfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        armconfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        armconfig.sensorTimeBase = SensorTimeBase.PerSecond;
        armconfig.sensorCoefficient = 360/4096;
        armconfig.magnetOffsetDegrees = (27.58);
        canCoder.configFactoryDefault();
        canCoder.configAllSettings(armconfig);

    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("zeroing init", isZeroed);
        SmartDashboard.putNumber("Arm CANCoder value", canCoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm Gearbox encoder value", encoder.getPosition());
    }
    public void zeroArm (){
        isZeroed = true;
        while (canCoder.getAbsolutePosition() < -15) {
            leader.set(-0.1);
        }
            encoder.setPosition(encoder.getPosition());
            leader.set(0.1);
            Timer.delay(.5);
            leader.set(0);
    }

    //intializing function
    public void start(){
        zeroArm();
    }

    //runs periodicaly in teleop
    public void updateStates(){
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

        m_setpoint = profile.calculate(0.02);
        //Gives sparkmax pid controller wpi feedfoward and state info
        PIDcontroller.setReference
            (m_setpoint.position, 
            ControlType.kPosition, 
    1, 
            m_feedforward.calculate(m_setpoint.velocity)/12.0);
    }

    //this function takes DEGREES
    public void moveTo(double position) {  
        position = position*(clicksToDegreesFactor) - 10;
            m_goal = new TrapezoidProfile.State(position, 0);
    }

    //add increments to a new position and waits before doing again
    public void moveByIncrements(double increments, double updateTime) {
        double lastPosition = m_setpoint.position*(clicksToDegreesFactor) - 10;
        m_goal = new TrapezoidProfile.State(lastPosition + increments, 0);   
        Timer.delay(updateTime);

    //if the "setPositionPIDWrappingMinInput()" or "setPositionPIDWrappingMaxInput()" function does not work in the constructor

        // if ((lastPosition + increments > Constants.Arm.maxPos) || (lastPosition + increments < Constants.Arm.minPos)){
        //     m_goal = new TrapezoidProfile.State(lastPosition, 0);   
        // } else {
        //     m_goal = new TrapezoidProfile.State(lastPosition + increments, 0);
        //     Timer.delay(updateTime);
        // }
    }

    public void moveByController(double controllerAxis, double speed) {
        //speed = degrees per second at maxiumum throttle
        moveByIncrements(controllerAxis, speed);
    } 
}

