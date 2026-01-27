// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: Spinner.java
// ************************************************************


package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Velocity;

public class Spinner extends SubsystemBase {
    private final TalonFX motor;
    
    /**
     * Contructor for Spinner class under subsystem
     * configures spinner motor and its voltage,etc
     * @param canID int
     */
    public Spinner(int canID) {
        motor = new TalonFX(canID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode response = motor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }  
    }

    /**
     * Sets the motor speed to the given value
     * calls stopMotor if speed is set to 0
     * @param speed double
     */
    public void spin(double speed) {
       speed = MathUtil.clamp(speed, -1, 1);
       motor.set(speed);
    
        if(speed == 0){
            motor.stopMotor();
        }
         // speed = 0; // for testing
    }



    /**
     * Gets the velocity speed of motor measured in RPM
     * @return velocity speed as double (RPM)
     */
    public double getSpeedRpm(){

        //TODO: fix this. 
        // StatusSignal<velocit> velocityStatSig = motor.getVelocity().getValueAsDouble();
        // Velocity velocity = motor.getVelocity().getValue();
        // StatusSignal CTREAbomination = motor.getVelocity(true);
        // return CTREAbomination.getValueAsDouble();
        return motor.getVelocity(true).getValueAsDouble();
    }

    /**
     * Publishes telemetry data to the SmartDashboard of spinner
     * Publishes the spinner speed as double (RPM)
     */
    public void publishTelemetry(){
        SmartDashboard.putNumber("Spinner_Speed ", motor.getVelocity(true).getValueAsDouble());
    }

}
