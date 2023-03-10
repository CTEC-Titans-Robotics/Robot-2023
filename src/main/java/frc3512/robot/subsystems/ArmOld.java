// package frc3512.robot.subsystems;

// import com.ctre.phoenix.sensors.*;
// import com.revrobotics.*;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc3512.lib.util.CANCoderUtil;
// import frc3512.robot.Constants;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// public class ArmOld extends SubsystemBase {
//     public static final CANSparkMax followerMotor = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
//     public static final CANSparkMax leaderMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
//     public static final SparkMaxPIDController mainPIDMotor = leaderMotor.getPIDController();
//     private static final ArmFeedforward armFeedForward = new ArmFeedforward(0.081228,0.11788,4.8424, 0.076763);
//     public static final SparkMaxAbsoluteEncoder GearboxEncoder = leaderMotor.getAbsoluteEncoder(Type.kDutyCycle);
//     public static final CANCoder topEncoder = new CANCoder(7);

//     private static double kP = .01; // 0000011903
//     private static double kI = 0; // 0000010902
//     private static double kD = 0.01;
//     private static double kIz = 0;
//     private static double kFF;
//     private static double kMaxOutput = 1;
//     private static double kMinOutput = -1;
//     private static double minVel = -0.1;
//     private static double maxVel = 0.1;
//     private static double maxAccel = 0;
//     private static double minPos = -72;
//     private static double maxPos = 10;
//     private static double angleCANOffset = 27.58;

//     private static SparkMaxLimitSwitch limitSwitch = leaderMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

//     public static double armAngle;
//     public boolean reachedMax;
//     public boolean reachedMin;
//     public final BooleanSupplier reachedMaxSup = () -> reachedMax; 
//     public final BooleanSupplier reachedMinSup = () -> reachedMin;
    


//     public ArmOld() {
        
//         SmartDashboard.putBoolean("zeroing init", false);

//         initDashboard();
//         followerMotor.follow(leaderMotor);
//         armConfigAngleEncoder();
        
    
//         mainPIDMotor.setP(kP);
//         mainPIDMotor.setI(kI);
//         mainPIDMotor.setD(kD);
//         mainPIDMotor.setIZone(kIz);
//         mainPIDMotor.setFF(kFF);

//         mainPIDMotor.setPositionPIDWrappingEnabled(true);
//         mainPIDMotor.setPositionPIDWrappingMaxInput(maxPos);
//         mainPIDMotor.setPositionPIDWrappingMinInput(minPos);

//         GearboxEncoder.setPositionConversionFactor(1);
//         // mainPIDMotor.setFeedbackDevice(GearboxEncoder);
        
//     }

//     public void moveToPos(DoubleSupplier position, DoubleSupplier vel, DoubleSupplier accel) {
//         double defaultVel = 0.7;
//         double defaultAccel = 0;
//         REVLibError res = mainPIDMotor.setFF(armFeedForward.calculate((position.getAsDouble())*(Math.PI/180),
//                 vel == null ? defaultVel : vel.getAsDouble(),
//                 accel == null ? defaultAccel : accel.getAsDouble()));
//         if(res.value != REVLibError.kOk.value) {
//             System.out.println(new IllegalStateException().getMessage());
//         }
//     }

//     // Accel and Vel can be null
//     public Command move(DoubleSupplier position, DoubleSupplier vel, DoubleSupplier accel) {
//         return run(() -> moveToPos(position, vel, accel));
//     }

// private void armConfigAngleEncoder() {
//     CANCoderConfiguration armconfig = new CANCoderConfiguration();
//     armconfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
//     armconfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//     armconfig.sensorTimeBase = SensorTimeBase.PerSecond;
//     armconfig.sensorCoefficient = 0.087890625;

//     topEncoder.configFactoryDefault();
//     topEncoder.configAllSettings(armconfig);
//     CANCoderUtil.setCANCoderBusUsage(topEncoder, CANCoderUtil.CANCoderUsage.kMinimal);

//     topEncoder.setPositionToAbsolute();
//     followerMotor.setIdleMode(IdleMode.kBrake);
//     leaderMotor.setIdleMode(IdleMode.kBrake);
//   }

//     public static void initDashboard() {
//         // display PID coefficients on SmartDashboard
//         SmartDashboard.putNumber("P Gain", kP);
//         SmartDashboard.putNumber("I Gain", kI);
//         SmartDashboard.putNumber("D Gain", kD);
//         SmartDashboard.putNumber("I Zone", kIz);
//         SmartDashboard.putNumber("Feed Forward", kFF);
//         SmartDashboard.putNumber("Max Output", kMaxOutput);
//         SmartDashboard.putNumber("Min Output", kMinOutput);

//         // display Smart Motion coefficients
//         SmartDashboard.putNumber("Max Velocity", maxVel);
//         SmartDashboard.putNumber("Min Velocity", minVel);
//         SmartDashboard.putNumber("Max Acceleration", maxAccel);
//         SmartDashboard.putNumber("Set Position", 0);
//         SmartDashboard.putNumber("Set Velocity", 0);


//         // button to toggle between velocity and smart motion modes
//         SmartDashboard.putBoolean("Mode", true);

//         //encoder value
//         double armAngle = topEncoder.getAbsolutePosition() - angleCANOffset;
//         SmartDashboard.putNumber("Rotation Encoder", topEncoder.getAbsolutePosition());
//         SmartDashboard.putNumber("Arm Angle", armAngle);
//     }
    
//     public void periodic() {
//         initDashboard();
//         double armAngle = topEncoder.getAbsolutePosition() - angleCANOffset;
//         SmartDashboard.putNumber("Rotation Encoder", topEncoder.getAbsolutePosition());
//         SmartDashboard.putNumber("Arm Angle", armAngle);
//         SmartDashboard.putBoolean("reachedMax", reachedMax);

//         if (armAngle > maxPos) {
//             reachedMax = true;
//         } else {
//             reachedMax = false;
//         }

//         if (armAngle < minPos || limitSwitch.isPressed()) {
//             reachedMin = true;
//         } else {
//             reachedMin = false;
//         }
    
//         //PID CODE START

//     //     // read PID coefficients from SmartDashboard
//     //     double p = SmartDashboard.getNumber("P Gain", 0);
//     //     double i = SmartDashboard.getNumber("I Gain", 0);
//     //     double d = SmartDashboard.getNumber("D Gain", 0);
//     //     double iz = SmartDashboard.getNumber("I Zone", 0);
//     //     double ff = SmartDashboard.getNumber("Feed Forward", 0);
//     //     double max = SmartDashboard.getNumber("Max Output", 0);
//     //     double min = SmartDashboard.getNumber("Min Output", 0);
//     //     double maxV = SmartDashboard.getNumber("Max Velocity", 0);
//     //     double minV = SmartDashboard.getNumber("Min Velocity", 0);
//     //     double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
//     //     double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
        

//     //     // if PID coefficients on SmartDashboard have changed, write new values to controller
//     //     if((p != kP)) { mainPIDMotor.setP(p); kP = p; }
//     //     if((i != kI)) { mainPIDMotor.setI(i); kI = i; }
//     //     if((d != kD)) { mainPIDMotor.setD(d); kD = d; }
//     //     if((iz != kIz)) { mainPIDMotor.setIZone(iz); kIz = iz; }
//     //     if((ff != kFF)) { mainPIDMotor.setFF(ff); kFF = ff; }
//     //     if((max != kMaxOutput) || (min != kMinOutput)) {
//     //         mainPIDMotor.setOutputRange(min, max);
//     //         kMinOutput = min; kMaxOutput = max;
//     //     }
//     //     if((maxV != maxVel)) { mainPIDMotor.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
//     //     if((minV != minVel)) { mainPIDMotor.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
//     //     if((maxA != maxAccel)) { mainPIDMotor.setSmartMotionMaxAccel(maxA,0); maxAccel = maxA; }
        
//     //     double setPoint, processVariable;
//     //     boolean mode = SmartDashboard.getBoolean("Mode", false);
//     //     if(mode) {
//     //         setPoint = SmartDashboard.getNumber("Set Velocity", 0);
//     //         // mainPIDMotor.setReferen/ce(setPoint, CANSparkMax.ControlType.kVelocity);
//     //     } else {
//     //         setPoint = SmartDashboard.getNumber("Set Position", 0);
//     //         /*
//     //          * As with other PID modes, Smart Motion is set by calling the
//     //          * setReference method on an existing pid object and setting
//     //          * the control type to kSmartMotion
//     //           */
//     //         // mainPIDMotor.setReference(setPoint, CANSparkMax.ControlType.kPosition);
            
//     //     } 

//     //     // PID CODE END

//      } 

//     public void zeroingProtocol() {
//          SmartDashboard.putBoolean("zeroing init", true);
//         limitSwitch.enableLimitSwitch(false);
//         while (!limitSwitch.isPressed()) {
//             leaderMotor.set(-0.1);
//         }
//             GearboxEncoder.setZeroOffset(GearboxEncoder.getPosition());
//             leaderMotor.set(0.1);
//             Timer.delay(.5);
//             leaderMotor.set(0);
//             mainPIDMotor.setReference(10, CANSparkMax.ControlType.kPosition);
//       }

//     public void simpleArmPositiveMovement(BooleanSupplier max){
//         if(!max.getAsBoolean()) {
//             leaderMotor.set(0.4);
//         } else {
//             stopMovement();
//         }
//     }
//     public void simpleArmNegativeMovement(BooleanSupplier min){
//         if(!min.getAsBoolean()) {
//             leaderMotor.set(-0.4);
//         } else {
//             stopMovement();
//         }
//     }

//     public void stopMovement() {
//         leaderMotor.set(-0.01);
//     }

//     public Command stopMovementCommand() {
//         return run(() -> leaderMotor.set(0));
//     }
// }