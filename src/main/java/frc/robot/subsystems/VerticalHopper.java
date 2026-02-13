package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    private double accelerationMetersPerSecSquared = 10; // m/s^2 (target linear acceleration)

    private double targetVelocityMetersPerSec = 11.25; // target linear velocity (m/s)
    private double kV = 0.1131;   
    private double kA = 0.00;
    private double kP = 0.07;
    private double kI = 0.00;
    private double kD = 0.00;
    private final Slot0Configs slot0Configs;
    private final TalonFXConfiguration motorConfig;

    public VerticalHopper() {
        hoppervertical = new TalonFX(0); 
        final TalonFXConfiguration configsK = new TalonFXConfiguration();

        slot0Configs = configsK.Slot0;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);
        motorConfig.MotionMagic =
        new MotionMagicConfigs().withMotionMagicAcceleration(Units.radiansToRotations(accelerationMetersPerSecSquared / Units.inchesToMeters(1)));
        motorConfig.Slot0 = slot0Configs; 
        hoppervertical.getConfigurator().apply(slot0Configs);
        hoppervertical.getConfigurator().apply(motorConfig);
        voltRequest = new MotionMagicVelocityVoltage(0); 
    }

    public void revAtVelocity(double velocityMetersPerSec) {
        double radsPerSec = velocityMetersPerSec / Units.inchesToMeters(1); // v = w * r -> w = v / r
        double rotsPerSec = Units.radiansToRotations(radsPerSec); // rad/s -> rot/s
        hoppervertical.setControl(voltRequest.withVelocity(rotsPerSec)); // Motion Magic velocity target
    }
    public void rev() {
        double radsPerSec = targetVelocityMetersPerSec / Units.inchesToMeters(1);
        double rotsPerSec = Units.radiansToRotations(radsPerSec);
        hoppervertical.setControl(voltRequest.withVelocity(rotsPerSec)); // stored target velocity
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
    public double getkV() {return kV;}
    public double getkA() {return kA;}
    public double getkP() {return kP;}
    public double getkI() {return kI;}
    public double getkD() {return kD;}
    public double getProfileVelocity() {
        double rotsPerSec = hoppervertical.getClosedLoopReference().getValueAsDouble(); // motor rot/s
        double radsPerSec = Units.rotationsToRadians(rotsPerSec); // rot/s -> rad/s
        double metersPerSec = radsPerSec * Units.inchesToMeters(1); // v = w * r
        return metersPerSec;
    }
    public double getProfileAcceleration() {
        double rotsPerSecSquared = hoppervertical.getClosedLoopReferenceSlope().getValueAsDouble(); // motor rot/s^2
        double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared); // rot/s^2 -> rad/s^2
        double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(1); // a = alpha * r
        return metersPerSecSquared;
    }
    public void setkV(double value) {kV = value;}
    public void setkA(double value) {kA = value;}
    public void setkP(double value) {kP = value;}
    public void setkI(double value) {kI = value;}
    public void setkD(double value) {kD = value;}
    
    public void updatePID() {
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        motorConfig.Slot0 = slot0Configs;
        hoppervertical.getConfigurator().apply(motorConfig);
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Vertical Hopper");

        builder.addDoubleProperty("Velocity (m/s)", this::getVelocity, null);
        builder.addDoubleProperty("Acceleration (m/s^2)", this::getAcceleration, null);

        builder.addDoubleProperty("Target Acceleration (m/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
        builder.addDoubleProperty("Target Velocity (m/s)", this::getTargetVelocity, this::setTargetVelocity);
        
        builder.addDoubleProperty("Profile Velocity (m/s)", this::getProfileVelocity, null);
        builder.addDoubleProperty("Profile Acceleration (m/s^2)", this::getProfileAcceleration, null);

        builder.addDoubleProperty("kV", this::getkV, this::setkV);
        builder.addDoubleProperty("kA", this::getkA, this::setkA);
        builder.addDoubleProperty("kP", this::getkP, this::setkP);
        builder.addDoubleProperty("kI", this::getkI, this::setkI);
        builder.addDoubleProperty("kD", this::getkD, this::setkD);
        }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}