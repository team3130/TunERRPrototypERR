package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    //For Shooting Speed, ect..
    private double deltaXHub = 8; //meters
    private final double deltaXHeight = 2; //meters
    private double angle = 90;
    private double targetVelocityMetersPerSec = 15;
    private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));
    private double accelerationMetersPerSecSquared = 45;
    private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));
    private final double flyWheelSpeed = 1; // CHANGE FOR PERCENTAGE

    //Objects/Constants
    private final TalonFX talonWheelLeft;
    private final TalonFX talonWheelRight;
    private final TalonFX talonRightHood;
    private final TalonFX talonLeftHood;
    private final DigitalInput limitSwitch;
    private final double gearRatioHood = 1; //gearRatio * Code Rotations = Rotations IRL

    //Shooter is Slot: PID
    private final Slot0Configs slot0Configs;
    private double slot0kV = 0.12; 
    private double slot0kA = 0.01;
    private double slot0kP = 0; 
    private double slot0kI = 0;
    private double slot0kD = 0.0;
    private final MotionMagicVelocityVoltage voltRequest0;
    private final TalonFXConfiguration configM;

    //Hood is Slot0: PID
    private final Slot1Configs slot1Configs;
    //Hood is Slot1: PID
    private final double slot1kP = 1; 
    private final double slot1kI = 0;
    private final double slot1kD = 0.0;
    private final double targetVelocity = 0;
    private final double targetAcceleration = 1.2;
    private final MotionMagicDutyCycle voltRequest1;
    private final TalonFXConfiguration config;
    
    //Shooter Constructer
    public Shooter() {
        //Robot CAN ID
        talonLeftHood = new TalonFX(0);
        talonRightHood = new TalonFX(0);
        talonWheelLeft = new TalonFX(33);
        talonWheelRight = new TalonFX(32);
        this.limitSwitch = null;
        talonWheelRight.setControl(new Follower(talonWheelLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        talonRightHood.setControl(new Follower(talonLeftHood.getDeviceID(), MotorAlignmentValue.Opposed));
    
        //PID For Hood
        slot1Configs = new Slot1Configs();
        slot1Configs.kP = slot1kP;
        slot1Configs.kI = slot1kI;
        slot1Configs.kD = slot1kD;

        //Motor Config and Motion Magic for Hood
        config = new TalonFXConfiguration();
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        config.MotionMagic.withMotionMagicCruiseVelocity(targetVelocity).withMotionMagicAcceleration(targetAcceleration);
        config.Slot1 = slot1Configs;
        talonLeftHood.getConfigurator().apply(config);
        voltRequest1 = new MotionMagicDutyCycle(0);
        
        //PID for Shooter
        final TalonFXConfiguration configS = new TalonFXConfiguration();
        slot0Configs = configS.Slot0;
        slot0Configs.kA = slot0kA;
        slot0Configs.kV = slot0kV;
        slot0Configs.kP = slot0kP;
        slot0Configs.kI = slot0kI;
        slot0Configs.kD = slot0kD;

        //Motor Config and Motion Magic Shooter
        configM = new TalonFXConfiguration();
        configM.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);
        configM.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(accelerationRotations);
        configM.Slot0 = slot0Configs;
        talonWheelLeft.getConfigurator().apply(configM);
        voltRequest0 = new MotionMagicVelocityVoltage(0);
    }
    
    //Hood Methods
    //Set Angle to Zero
    public void angleZero() {
        if (limitSwitch.get()) {
            talonLeftHood.setPosition(90);
            angle = 90;
        }
    }

    //Method to Angle hood to any angle
    public void hoodAngler() {
        double angle = Math.PI/2;
        final double rotationHood = angle/(2*Math.PI);
        talonLeftHood.setControl(voltRequest1.withPosition(rotationHood * gearRatioHood));
    }

    public void hoodSet0() {
        talonLeftHood.setControl(voltRequest1.withPosition(0));
    }

    //Shooter Methods
    public void shootForward() {
        talonWheelLeft.setControl(voltRequest0.withVelocity(targetVelocityRotations));
    }

    public void shootInverted() {
        talonWheelLeft.setControl(voltRequest0.withVelocity(-targetVelocityRotations));
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

    public double getVelocity() {
        double rotsPerSec = talonWheelLeft.getVelocity().getValueAsDouble();
        double radsPerSec = Units.rotationsToRadians(rotsPerSec);
        double metersPerSec = radsPerSec * Units.inchesToMeters(2);
        return metersPerSec;
    }
    
    //Getters and setters for Target Velocity and Acceleration
    public double getTargetVelocity() {return targetVelocityMetersPerSec;}
    public double getTargetAcceleration() {return accelerationMetersPerSecSquared;}
    public void setTargetAcceleration(double value) {accelerationMetersPerSecSquared = value;}
    public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;}


    //Getters and setters for PID values
    public double getShooterkV() {return slot0kV;}
    public double getShooterkA() {return slot0kA;}
    public double getShooterkP() {return slot0kP;}
    public double getShooterkI() {return slot0kI;}
    public double getShooterkD() {return slot0kD;}
    public void setShooterkV(double value) {slot0kV = value;}
    public void setShooterkA(double value) {slot0kA = value;}
    public void setShooterkP(double value) {slot0kP = value;}
    public void setShooterkI(double value) {slot0kI = value;}
    public void setShooterkD(double value) {slot0kD = value;}

    public double getProfileVelocity() {
        double rotsPerSec = talonWheelLeft.getClosedLoopReference().getValueAsDouble();
        double radsPerSec = Units.rotationsToRadians(rotsPerSec);
        double metersPerSec = radsPerSec * Units.inchesToMeters(2);
        return metersPerSec;
    }

    public void updatePID() {
        slot0Configs.kV = slot0kV;
        slot0Configs.kA = slot0kA;
        slot0Configs.kP = slot0kP;
        slot0Configs.kI = slot0kI;
        slot0Configs.kD = slot0kD;
        configM.Slot0 = slot0Configs;
        talonWheelLeft.getConfigurator().apply(configM);
      }

    public double getProfileAcceleration() {
        double rotsPerSecSquared = talonWheelLeft.getClosedLoopReferenceSlope().getValueAsDouble();
        double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared);
        double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(2);
        return metersPerSecSquared;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.addDoubleProperty("Motor Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Target Acceleration", this::getTargetAcceleration, this::setTargetAcceleration);
        builder.addDoubleProperty("Target Velocity", this::getTargetVelocity, this::setTargetVelocity);
        builder.addDoubleProperty("Profile Velocity", this::getProfileVelocity, null);
        builder.addDoubleProperty("Profile Acceleration", this::getProfileAcceleration, null);

        builder.addDoubleProperty("Shooter kV", this::getShooterkV, this::setShooterkV);
        builder.addDoubleProperty("Shooter kA", this::getShooterkA, this::setShooterkA);
        builder.addDoubleProperty("Shooter kP", this::getShooterkP, this::setShooterkP);
        builder.addDoubleProperty("Shooter kI", this::getShooterkI, this::setShooterkI);
        builder.addDoubleProperty("Shooter kD", this::getShooterkD, this::setShooterkD);

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //angleZero();
    }
}
