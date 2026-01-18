package frc.robot.subsystems;

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
import frc.robot.constants.MotorConfigs;

public class FlyWheel extends SubsystemBase {
    // Constants for PID/Feedforward
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KS = 0.2;
    private final double KV = 0.1;

    private final String CAN_BUS = "rio";
    private final String MOTOR_TYPE = "Falcon500";
    private final double SHOOTING_RPS = 30.0;
    private double targetRPS = 0.0; // Positive = Counter Clockwise, Negative = Clockwise, initially at rest

    private TalonFX m_upper, m_lower;
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public FlyWheel() {
        this.m_lower = new TalonFX(1, CAN_BUS);
        this.m_upper = new TalonFX(2, CAN_BUS);
        configureMotors();
    }

    private void configureMotors() {
        var motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast,
                                InvertedValue.CounterClockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));

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
     * Get the velocity of the upper motor in Revolution per Second
     * 
     * @return the velocity of the upper motor
     */
    public double getVelocityRPS() {
        return m_upper.getVelocity().getValueAsDouble();
    }

    /**
     * Set the control of a specific motor
     * 
     * @param req
     *            - the control request
     */
    private void setControl(ControlRequest req) {
        if (m_upper.isAlive()) {
            m_upper.setControl(req);
        }
    }

    /**
     * Stop the 2 motors
     */
    public void stopMotor() {
        targetRPS = 0.0;
    }

    public void startShooting() {
        targetRPS = SHOOTING_RPS;
    }

    @Override
    public void periodic() {
        // Update the motor's velocity
        m_velocityRequest.Velocity = targetRPS;
        setControl(m_velocityRequest);

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
