package frc.theRobot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Spinner extends SubsystemBase {
    //configurtition for the motor begins here
    private TalonFX motorTalon;
    //velcity and voltage control
    private VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    // values found through manual system characterization using Phoenix Tuner X.
    private Slot0Configs motorRpmGains = new Slot0Configs().withKS(0.119956).withKV(0.10905).withKP(0.35).withKD(0.0).withKI(0.0);
    private double speedRpm = 0.0;
    private NeutralModeValue motorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;

    public Spinner(){
        motorTalon = new TalonFX(Constants.motorCanID);
        configureMotor();
        System.out.print("Motor configuration complete");
        motorTalonVelocityController.withSlot(0).withEnableFOC(true);
        motorTalonVelocityController.withUpdateFreqHz(0); // send commands instantly
    }

    public void spinMotor(double speedRpm){
        //Sets speed to a specific state for testing
        this.speedRpm = speedRpm;
        System.out.println("Motor speed set to " + speedRpm);
    }

    public void motorStop(){
        //stop voltage
        this.speedRpm = 0;
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
        System.out.println("Motor stopped");
    }
    
    public double getSpeedRpm(){
        //return speed
        return motorTalon.getVelocity().getValueAsDouble()*60;
    }

    private void configureMotor(){
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration(); 
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorTargetNeutralModeValue;
        motorTalonMotorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
        motorTalonMotorConfiguration.Slot0 = motorRpmGains;
        motorTalonMotorConfiguration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;
        motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        StatusCode response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + motorTalon.getDeviceID() + " failed config with error " + response.toString());
        }
    
    }

    public void periodic(){
        SmartDashboard.putNumber("Spinner Speed", this.getSpeedRpm());
        if(speedRpm != 0){
            motorTalon.setControl(this.motorTalonVelocityController.withVelocity(speedRpm/60));
        }
    }
}
