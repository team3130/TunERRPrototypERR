package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hoppervertical extends SubsystemBase {
    private final TalonFX hoppervertical;
    private double verticalSpeed = 0.5;

    public Hoppervertical() {
        hoppervertical = new TalonFX(Constants.CAN.Talon3); // not necessary but just in case
        hoppervertical.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }  

    public void runHoppervertical() {
        hoppervertical.set(verticalSpeed);
    }

    public void reverseHoppervertical() {
        hoppervertical.set(-verticalSpeed);
    }

    public void stopHoppervertical() {
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
