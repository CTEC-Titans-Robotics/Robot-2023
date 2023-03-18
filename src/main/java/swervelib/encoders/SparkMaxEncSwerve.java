package swervelib.encoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class SparkMaxEncSwerve extends SwerveAbsoluteEncoder {
    SparkMaxAbsoluteEncoder encoder;

    public SparkMaxEncSwerve(CANSparkMax motor) {
        encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);
    }

    @Override
    public void factoryDefault() {
    }

    @Override
    public void clearStickyFaults() {
    }

    @Override
    public void configure(boolean inverted) {
        encoder.setInverted(inverted);
    }

    @Override
    public double getAbsolutePosition() {
        return encoder.getPosition();
    }

    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }
}
