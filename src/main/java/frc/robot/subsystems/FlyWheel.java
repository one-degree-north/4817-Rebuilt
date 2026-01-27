package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlyWheelConstants;

public class FlyWheel extends SubsystemBase {
    // Constants for PID/Feedforward
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KS = 0.2;
    private final double KV = 0.1;

    private double targetRPS = 0.0; // Positive = Counter Clockwise, Negative = Clockwise, initially at rest

    private TalonFX m_upper, m_lower;
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public FlyWheel() {
        this.m_lower = new TalonFX(1);
        this.m_upper = new TalonFX(2);
        configureMotors();
    }

    private void configureMotors() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withStatorCurrentLimit(50.0);
        currentLimitsConfigs.SupplyCurrentLimit = 30.0;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = 1 / 1;

        var motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitsConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withFeedback(feedbackConfigs);

        var slot0Configs = motorConfigs.Slot0;
        slot0Configs.kP = KP; // Proportional gain
        slot0Configs.kI = KI; // Integral gain
        slot0Configs.kD = KD; // Derivative gain
        slot0Configs.kS = KS; // Static feedforward in volts
        slot0Configs.kV = KV; // Velocity feedforward (Volts / RPS)

        m_upper.getConfigurator().apply(motorConfigs);
        m_lower.getConfigurator().apply(motorConfigs);
        m_lower.setControl(new Follower(m_upper.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Get the velocity of the 2 motor sin Revolution per Second
     * 
     * @return the velocity of the 2 motors
     */
    private double getVelocityRPS() {
        return m_upper.getVelocity().getValueAsDouble();
    }

    /**
     * Set the control of the 2 motors
     * 
     * @param request - the control request
     */
    private void setControl(ControlRequest request) {
        if (m_upper.isAlive()) {
            m_upper.setControl(request);
        }
    }

    /**
     * Check if the current motors' RPS is within the tolerance range compare to the
     * targetRPS
     * 
     * @return true if the motors are in the range, false otherwise
     */
    private boolean atSetpoint() {
        return Math.abs(getVelocityRPS() - targetRPS) <= FlyWheelConstants.TOLERANCE;
    }

    /**
     * Check if the current motors' RPS is within the tolerance range compare to the
     * targetRPS
     * 
     * @param withTolerance - whether to include tolerance or not
     * @return true if the motors are within the expected range or matches with the
     *         setpoint value
     */
    public boolean atSetpoint(boolean withTolerance) {
        if (withTolerance)
            return atSetpoint();
        return Double.compare(getVelocityRPS(), targetRPS) == 0;
    }

    /**
     * Set the Revolution-per-Second value of the 2 motors
     * 
     * @param RPS - The Revolution-per-Second value in decimals
     */
    public void setTargetRPS(double RPS) {
        targetRPS = RPS;
    }

    @Override
    public void periodic() {
        // Update the motor's velocity
        if (!atSetpoint()) {
            m_velocityRequest.Velocity = targetRPS;
            setControl(m_velocityRequest);
        }

        // Display important information on SmartDashboard
        SmartDashboard.putNumber("Motor Velocity (RPS)", getVelocityRPS());
        SmartDashboard.putNumber("Target Velocity (RPS)", targetRPS);
        SmartDashboard.putNumber("KP Value", KP);
        SmartDashboard.putNumber("KI Value", KI);
        SmartDashboard.putNumber("KD Value", KD);
        SmartDashboard.putNumber("KS Value", KS);
        SmartDashboard.putNumber("KV Value", KV);
    }
}
