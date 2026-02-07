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
    private double accelerationMetersPerSecSquared = 49;
    private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));

    private double targetVelocityMetersPerSec = 11.25;
    private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));

    public VerticalHopper() {
        hoppervertical = new TalonFX(34); // not necessary but just in case
        hoppervertical.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)));
        voltRequest = new MotionMagicVelocityVoltage(0);
    }
    
    public void revAtVelocity(double velocityMetersPerSec) {
        double radsPerSec = velocityMetersPerSec / Units.inchesToMeters(2);
        double rotsPerSec = Units.radiansToRotations(radsPerSec);
        hoppervertical.setControl(voltRequest.withVelocity(rotsPerSec));  
    }
    public void rev() {
        hoppervertical.setControl(voltRequest.withVelocity(targetVelocityRotations));
    }

    public double getVelocity() {
        double rotsPerSec = hoppervertical.getVelocity().getValueAsDouble();
        double radsPerSec = Units.rotationsToRadians(rotsPerSec);
        double metersPerSec = radsPerSec * Units.inchesToMeters(2);
        return metersPerSec;
    }
    public double getAcceleration() {
        double rotsPerSecSquared = hoppervertical.getAcceleration().getValueAsDouble();
        double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared);
        double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(2);
        return metersPerSecSquared;
    }  
    public double getTargetAcceleration() {return accelerationMetersPerSecSquared;}
    public void setTargetAcceleration(double value) {accelerationMetersPerSecSquared = value;}

    public double getTargetVelocity() {return targetVelocityMetersPerSec;}
    public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;}
    
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
