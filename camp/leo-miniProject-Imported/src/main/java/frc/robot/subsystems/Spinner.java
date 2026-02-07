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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Velocity;

public class Spinner extends SubsystemBase {
    private final TalonFX motor;

    private final VelocityVoltage controller = new VelocityVoltage(0.0);
    private double targetRPS = 0.0;

    //private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.1199563795).withKV(0.1090512541).withKP(0.4).withKD(0.0);

    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.1199563795).withKV(0.1090512541).withKP(0.2);
    /**
     * Contructor for Spinner class under subsystem
     * configures spinner motor and its voltage,etc
     * 
     * @param canID int
     */
    public Spinner(int canID) {
        motor = new TalonFX(canID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = slot0Configs;

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

    public void setRPM(double rpm) {
        this.targetRPS = rpm / 60.0;
    }

    public double getRPM() {
        return this.motor.getVelocity().getValueAsDouble() * 60.0;
    }

    public void stop() {
        targetRPS = 0.0;
        this.motor.stopMotor();
    }

    @Override
    public void periodic() {
        controller.withVelocity(this.targetRPS);
        motor.setControl(controller);
    }

    /**
     * Publishes telemetry data to the SmartDashboard of spinner
     * Publishes the spinner speed as double (RPM)
     */
    public void publishTelemetry() {
        SmartDashboard.putNumber("Spinner_Speed ", motor.getVelocity(true).getValueAsDouble());
    }

}
