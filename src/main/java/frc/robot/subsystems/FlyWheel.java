package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

public class FlyWheel extends SubsystemBase {
    // Constants for PID/Feedforward
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.2;
    private final double kV = 0.1;

    private final String CAN_BUS = "rio";
    private final String MOTOR_TYPE = "Falcon500";
    private final double SHOOTING_RPS = 30.0; // Positive = Counter Clockwise, Negative = Clockwise
    private boolean isShooting = false;

    private final TalonFX m_upper, m_lower;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageOut;

    public FlyWheel() {
        this.m_lower = new TalonFX(1, CAN_BUS);
        this.m_upper = new TalonFX(2, CAN_BUS);
        this.voltageOut = new VoltageOut(0);
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
        slot0Configs.kP = kP; // Proportional gain
        slot0Configs.kI = kI; // Integral gain
        slot0Configs.kD = kD; // Derivative gain
        slot0Configs.kS = kS; // Static feedforward in volts
        slot0Configs.kV = kV; // Velocity feedforward (Volts / RPS)

        m_upper.getConfigurator().apply(motorConfigs);
        m_lower.getConfigurator().apply(motorConfigs);
        m_lower.setControl(new Follower(m_upper.getDeviceID(), MotorAlignmentValue.Aligned));
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
        isShooting = false;
        setControl(voltageOut.withOutput(0.0));
    }

    /**
     * Get the velocity of the upper motor in Revolution per Second
     * 
     * @return the velocity of the upper motor
     */
    public double getVelocityRPS() {
        return m_upper.getVelocity().getValueAsDouble();
    }

    public void run() {
        isShooting = true;
    }

    @Override
    public void periodic() {
        if (isShooting) {
            m_velocityRequest.Velocity = SHOOTING_RPS;
            setControl(m_velocityRequest);
        }

        // Display important information on SmartDashboard
        SmartDashboard.putNumber("Motor Velocity (RPS)", m_velocityRequest.Velocity);
        SmartDashboard.putNumber("Target Velocity (RPS)", SHOOTING_RPS);
        SmartDashboard.putNumber("KP Value", kP);
        SmartDashboard.putNumber("KI Value", kI);
        SmartDashboard.putNumber("KD Value", kD);
        SmartDashboard.putNumber("KS Value", kS);
        SmartDashboard.putNumber("KV Value", kV);
    }
}
