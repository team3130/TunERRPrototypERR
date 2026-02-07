package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // unnecessary, but not an issue
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class VerticalHopper extends SubsystemBase {
    private final TalonFX hoppervertical;
    private final MotionMagicVelocityVoltage voltRequest;
    private double verticalSpeed = 0.3;
    private double accelerationMetersPerSecSquared = 49; // m/s^2 (target linear acceleration)
    private final double accelerationRotations =
          Units.radiansToRotations(accelerationMetersPerSecSquared / Units.inchesToMeters(1)); // convert to rot/s^2 (angular accerelation)

    private double targetVelocityMetersPerSec = 11.25; // target linear velocity (m/s)
    private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec / Units.inchesToMeters(1)); // convert to rot/s

    public VerticalHopper() {
        hoppervertical = new TalonFX(34);
        hoppervertical.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)));
        voltRequest = new MotionMagicVelocityVoltage(0); 
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
    }

    public void revAtVelocity(double velocityMetersPerSec) {
        double radsPerSec = velocityMetersPerSec / Units.inchesToMeters(1); // v = w * r -> w = v / r
        double rotsPerSec = Units.radiansToRotations(radsPerSec); // rad/s -> rot/s
        hoppervertical.setControl(voltRequest.withVelocity(rotsPerSec)); // Motion Magic velocity target
    }
    public void rev() {
        hoppervertical.setControl(voltRequest.withVelocity(targetVelocityRotations)); // stored target velocity
    }

    public double getVelocity() {
        double rotsPerSec = hoppervertical.getVelocity().getValueAsDouble(); // motor rot/s
        double radsPerSec = Units.rotationsToRadians(rotsPerSec); // rot/s -> rad/s
        double metersPerSec = radsPerSec * Units.inchesToMeters(1); // v = w * r
        return metersPerSec;
    }
    public double getAcceleration() {
        double rotsPerSecSquared = hoppervertical.getAcceleration().getValueAsDouble(); // motor rot/s^2
        double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared); // rot/s^2 -> rad/s^2
        double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(1); // a = alpha * r
        return metersPerSecSquared;
    }  
    public double getTargetAcceleration() {return accelerationMetersPerSecSquared;} // m/s^2 target accel
    public void setTargetAcceleration(double value) {accelerationMetersPerSecSquared = value;} // update target accel

    public double getTargetVelocity() {return targetVelocityMetersPerSec;} // m/s target velocity
    public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;} // update target velocity
    
    public void runHopperVertical() {
        hoppervertical.set(verticalSpeed);
    } 
    public void reverseHopperVertical() {
        hoppervertical.set(-verticalSpeed);
    }

    public void stopHopperVertical() {
        hoppervertical.set(0);
    }

    public double getVerticalSpeed() {
        return verticalSpeed;
    }

    public void setVerticalSpeed(double value) {
        verticalSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
