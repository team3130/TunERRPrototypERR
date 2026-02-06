package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // unnecessary, but not an issue
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor;

    private final MotionMagicVelocityVoltage voltRequest;
    
    private double hopperSpeed = 0.3;
    private double accelerationMetersPerSecSquared = 49;
     private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));

    private double targetVelocityMetersPerSec = 11.25;
    private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));
    public Hopper() {
        hopperMotor = new TalonFX(35); // not necessary but just in case
        hopperMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive)));
        voltRequest = new MotionMagicVelocityVoltage(0);
    }  

    public void revAtVelocity(double velocityMetersPerSec) {
        double radsPerSec = velocityMetersPerSec / Units.inchesToMeters(2);
        double rotsPerSec = Units.radiansToRotations(radsPerSec);
        hopperMotor.setControl(voltRequest.withVelocity(rotsPerSec));
      }
    
      public void rev() {
        rightShooter.setControl(voltRequest.withVelocity(targetVelocityRotations));
      }

    public void runHopper() {
        hopperMotor.set(hopperSpeed);
    }

    public void reverseHopper() {
        hopperMotor.set(-hopperSpeed);
    }

    public void stopHopper() {
        hopperMotor.set(0);
    }

    public double getHopperSpeed() {
        return hopperSpeed;
    }


    public void setHopperSpeed(double value) {
        hopperSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
