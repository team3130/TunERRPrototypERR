package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    //For AutoShoot
    private double deltaXHub = 8; //meters
    private double deltaXHeight = 2; //meters


    //Objects/Constants
    private TalonFX talonWheel;
    private TalonFX talonHood;
    private DigitalInput limitSwitch;
    private boolean limitSwitchValue = limitSwitch.get();
    private double velocityFuel = 11; //m/s
    private double gearRatioHood = 1; //gearRatio * Code Rotations = Rotations IRL
    

    //Shooter is Slot1: PID
    private VelocityVoltage voltRequest1;
    private Slot1Configs slot1Configs;
    private double slot1kP = 1; 
    private double slot1kI = 0;
    private double slot1kD = 0.0;
    private PositionVoltage m_request1;

    //Hood is Slot0: PID
    private double slot0kP = 1; 
    private double slot0kI = 0;
    private double slot0kD = 0.0;

    //MagicMotion Stuff for Hood
    private final MotionMagicDutyCycle voltRequest0;
    private double targetVelocity = 0;
    private double targetAcceleration = 1.2;
    private Slot0Configs slot0Configs;
    private PositionVoltage m_request0;
    private TalonFXConfiguration config;
    
    //Shooter Constructer
    public Shooter() {
        //PID For Hood
        voltRequest0 = new MotionMagicDutyCycle(0);
        slot0Configs = new Slot0Configs();
        m_request0 = new PositionVoltage(0).withSlot(0);
        slot0Configs.kP = slot0kP;
        slot0Configs.kI = slot0kI;
        slot0Configs.kD = slot0kD;
        config = new TalonFXConfiguration();
        config.MotionMagic.withMotionMagicCruiseVelocity(targetVelocity).withMotionMagicAcceleration(targetAcceleration);
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        talonHood.getConfigurator().apply(config);
        
        
        //PID for Shooter
        voltRequest1 = new VelocityVoltage(0).withSlot(1);
        m_request1 = new PositionVoltage(0).withSlot(1);
        slot1Configs = new Slot1Configs();
        slot1Configs.kP = slot1kP;
        slot1Configs.kI = slot1kI;
        slot1Configs.kD = slot1kD;
        talonWheel.getConfigurator().apply(slot1Configs);
    }
    
    //Set Angle to Zero
    public void angleZero() {
        if (limitSwitchValue = true) {
            m_request0 = new PositionVoltage(0).withSlot(0);
        }
    }

    //Method to Angle hood to any angle
    public void hoodAngler(double angle) {
        angleZero();
        //radian
        final double rotationHood = angle/(2*Math.PI);
        talonHood.setControl(voltRequest0.withPosition(rotationHood * gearRatioHood));
    }

    public void hoodSet0() {
        angleZero();
        talonHood.setControl(voltRequest0.withPosition(0));
    }

    public void shootForward() {
        angleZero();
        talonWheel.setControl(m_request1.withVelocity(velocityFuel).withFeedForward(0.0));
    }

    public void shootInverted() {
        angleZero();
        talonWheel.setControl(m_request1.withVelocity(-velocityFuel).withFeedForward(0.0));
    }

    public void shootStop() {
        angleZero();
        talonWheel.setControl(m_request1.withVelocity(0).withFeedForward(0.0));
    }


    //The Pre-reqs for this function are atleast 1 meter away from the hub and less than 8 meters way.
    public void autoAim() {
        angleZero();
        double num = -(deltaXHeight + (9.8 * deltaXHub * deltaXHub) / (velocityFuel * velocityFuel));
        double denom = Math.sqrt(deltaXHeight * deltaXHeight + deltaXHub * deltaXHub);
        double part1 = 2 * Math.PI;
        double part2 = Math.acos(num / denom);
        double part3 = Math.atan(deltaXHub / deltaXHeight);
        double radians =  0.5 * (part1 - part2 - part3);
        hoodAngler(radians);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
