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

public class Spinner extends SubsystemBase {
    private final TalonFX motor;
    
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

    public void spin(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        motor.set(speed);

        if(speed == 0){
            motor.stopMotor();
        }
    }

    public void motorStop(){
        motor.stopMotor();
    } 


    public double getSpeedRpm(){
        StatusSignal CTREAbomination = motor.getVelocity(true);
        return CTREAbomination.getValueAsDouble();
    }

    public void publishTelemetry(){
        SmartDashboard.putNumber("Spinner_Speed", motor.getVelocity(true).getValueAsDouble());
    }

}
