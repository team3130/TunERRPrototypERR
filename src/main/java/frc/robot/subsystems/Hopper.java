package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase {
    private final TalonFX hoppervertical;
    private final TalonFX hopperMotor;
    private double verticalSpeed = 0.5;
    private double hopperSpeed = 0.5;
    public Hopper() {
        hopperMotor = new TalonFX(35); // not necessary but just in case
        hopperMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
        hoppervertical = new TalonFX(34); // not necessary but just in case
        hoppervertical.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
    }  

    public void runHopper() {
        hopperMotor.set(hopperSpeed);
    }

    public void reverseHopper() {
        hopperMotor.set(-hopperSpeed);
    }
    public void runHopperVertical() {
        hoppervertical.set(verticalSpeed);
    } 
    public void reverseHopperVertical() {
        hoppervertical.set(-verticalSpeed);
    }


    public void stopHopper() {
        hopperMotor.set(0);
    }
    public void stopHopperVertical() {
        hoppervertical.set(0);
    }

    public double getHopperSpeed() {
        return hopperSpeed;
    }
    public double getVerticalSpeed() {
        return verticalSpeed;
    }


    public void setHopperSpeed(double value) {
        hopperSpeed = value;
    }
    public void setVerticalSpeed(double value) {
        verticalSpeed = value;
    }

    @Override
    public void periodic() {
        // run every 20 millisecond
    }
}
