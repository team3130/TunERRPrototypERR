package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    //For AutoShoot
    private double deltaXHub = 8; //meters
    private final double deltaXHeight = 2; //meters
    private double angle = 90;


    //Objects/Constants
    private final TalonFX talonWheelLeft;
    private final TalonFX talonWheelRight;
    private final TalonFX talonHood;
    private final DigitalInput limitSwitch;
    private final double flyWheelSpeed = 1; // CHANGE FOR PERCENTAGE
    private final double gearRatioHood = 1; //gearRatio * Code Rotations = Rotations IRL
    

    //Shooter is Slot: PID
    private final Slot0Configs slot0Configs;
    private double slot0kV = 0.12; 
    private double slot0kA = 0.01;
    private double slot0kP = 0.1; 
    private double slot0kI = 0;
    private double slot0kD = 0.0;
    private final MotionMagicVelocityVoltage voltRequest0;
    private final TalonFXConfiguration configS;

    //Hood is Slot1: PID
    private final double slot1kP = 1; 
    private final double slot1kI = 0;
    private final double slot1kD = 0.0;
    private final double targetVelocity = 0;
    private final double targetAcceleration = 1.2;
    private final MotionMagicDutyCycle voltRequest1;
    private final Slot1Configs slot1Configs;
    private final TalonFXConfiguration config;
    
    //Shooter Constructer
    public Shooter() {
        //Robot CAN ID
        talonHood = new TalonFX(0);
        talonWheelLeft = new TalonFX(33);
        talonWheelRight = new TalonFX(32);
        this.limitSwitch = null;
        talonWheelRight.setControl(new Follower(talonWheelLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    
        //PID For Hood
        voltRequest1 = new MotionMagicDutyCycle(0);
        slot1Configs = new Slot1Configs();
        slot1Configs.kP = slot1kP;
        slot1Configs.kI = slot1kI;
        slot1Configs.kD = slot1kD;
        config = new TalonFXConfiguration();
        config.MotionMagic.withMotionMagicCruiseVelocity(targetVelocity).withMotionMagicAcceleration(targetAcceleration);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        talonHood.getConfigurator().apply(config);
        
        
        //PID for Shooter
<<<<<<< Updated upstream
        configS = new TalonFXConfiguration();
        slot0Configs = new Slot0Configs();
        voltRequest0 = new MotionMagicVelocityVoltage(0);
=======
        final TalonFXConfiguration configS = new TalonFXConfiguration();
        slot0Configs = configS.Slot0;
>>>>>>> Stashed changes
        slot0Configs.kA = slot0kA;
        slot0Configs.kV = slot0kV;
        slot0Configs.kP = slot0kP;
        slot0Configs.kI = slot0kI;
        slot0Configs.kD = slot0kD;
        m_request0 = new MotionMagicVelocityVoltage(0);
        talonWheelLeft.getConfigurator().apply(slot0Configs);
    }
    
    //Hood Methods
    //Set Angle to Zero
    public void angleZero() {
        if (limitSwitch.get()) {
            talonHood.setPosition(90);
            angle = 90;
        }
    }

    //Method to Angle hood to any angle
    public void hoodAngler() {
        double angle = Math.PI/2;
        //radian
        final double rotationHood = angle/(2*Math.PI);
        talonHood.setControl(voltRequest1.withPosition(rotationHood * gearRatioHood));
    }

    public void hoodSet0() {
        talonHood.setControl(voltRequest1.withPosition(0));
    }

    //Shooter Methods
    public void shootForward() {
        talonWheelLeft.setControl(voltRequest0.withVelocity(flyWheelSpeed));
    }

    public void shootInverted() {
        talonWheelLeft.setControl(voltRequest0.withVelocity(-flyWheelSpeed));
    }


    public void shootForwardBasic() {
        talonWheelLeft.set(-flyWheelSpeed);
    }

    public void shootInvertedBasic() {
        talonWheelLeft.set(flyWheelSpeed);
    }

    //Stop for all shooter commands
    public void shootStop() {
        talonWheelRight.set(0);
        talonWheelLeft.set(0);
    }


    //The Pre-reqs for this function are atleast 1 meter away from the hub and less than 8 meters way.
    public void autoAim() {
        double num = -(deltaXHeight + (9.8 * deltaXHub * deltaXHub) / (flyWheelSpeed * flyWheelSpeed));
        double denom = Math.sqrt(deltaXHeight * deltaXHeight + deltaXHub * deltaXHub);
        double part1 = 2 * Math.PI;
        double part2 = Math.acos(num / denom);
        double part3 = Math.atan(deltaXHub / deltaXHeight);
        double radians =  0.5 * (part1 - part2 - part3);
    }
    public double getVelocity() {
        return talonWheelLeft.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //angleZero();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("Velocity", this::getVelocity, null);
    }
}
