package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX m_left, m_right;
    private VoltageOut voltageOut;

    private double m_left_volts = 0.0;
    private double m_right_volts = 0.0;

    public Intake() {
        m_left = new TalonFX(5);
        m_right = new TalonFX(6);
        voltageOut = new VoltageOut(0.0);
        configureMotors();
    }

    private void configureMotors() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withStatorCurrentLimit(50.0);
        currentLimitsConfigs.SupplyCurrentLimit = 30.0;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = 1 / 1;

        var motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitsConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        m_left.getConfigurator().apply(motorConfigs);
        m_right.getConfigurator().apply(motorConfigs);
    }

    private double getVoltages(TalonFX motor) {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    private void setControl(TalonFX motor, ControlRequest request) {
        if (motor.isAlive()) {
            motor.setControl(request);
        }
    }

    public void setMLeftVolts(double volts) {
        m_left_volts = volts;
    }

    public void setMRightVolts(double volts) {
        m_right_volts = volts;
    }

    @Override
    public void periodic() {
        setControl(m_left, voltageOut.withOutput(m_left_volts));
        setControl(m_right, voltageOut.withOutput(m_right_volts));

        SmartDashboard.putNumber("Left Motor Target Volts", m_left_volts);
        SmartDashboard.putNumber("Right Motor Target Volts", m_right_volts);
        SmartDashboard.putNumber("Left Motor Actual Volts", getVoltages(m_left));
        SmartDashboard.putNumber("Right Motor Actual Volts", getVoltages(m_right));
    }
}
