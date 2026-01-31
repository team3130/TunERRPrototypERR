package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor;
    private double hopperSpeed = 1;
    public Hopper() {
        hopperMotor = new TalonFX(Constants.CAN.falcon1); // not necessary but just in case
        hopperMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
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
