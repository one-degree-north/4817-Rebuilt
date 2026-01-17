package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

public class FlyWheel extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    final private double TARGET_RPM = 3000.0; // do note that this is positive
    private TalonFX m_upper, m_lower;
    private PIDController pid = new PIDController(0, 0, 0);

    private VoltageOut voltageOut;

    /* The voltage output is 5.0 by default */
    private double outputVolts = 5.0;
    private boolean isShooting = false;

    public FlyWheel() {
        this.m_lower = new TalonFX(1, CAN_BUS);
        this.m_upper = new TalonFX(2, CAN_BUS);
        this.voltageOut = new VoltageOut(0);
        this.pid.setSetpoint(-TARGET_RPM);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));
        m_upper.getConfigurator().apply(motorConfigs);
        m_lower.getConfigurator().apply(motorConfigs);

        m_lower.setControl(new Follower(m_upper.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Set the control of a specific motor
     * 
     * @param motor
     *              - the motor you are trying to modify
     * @param req
     *              - the control request
     */
    private void setControl(TalonFX motor, ControlRequest req) {
        if (motor.isAlive()) {
            motor.setControl(req);
        }
    }

    /**
     * Stop the 2 motors
     */
    public void stopMotor() {
        isShooting = false;
        setControl(m_upper, voltageOut.withOutput(0));
    }

    /**
     * Get the velocity of the upper motor in Revolution per Second
     * 
     * @return the velocity of the upper motor
     */
    public double getVelocityRPM() {
        return m_upper.getVelocity().getValueAsDouble() * 60.0;
    }

    public void run() {
        // Positive = Clockwise, Negative = Counter Clockwise
        isShooting = true;
        setControl(m_upper, voltageOut.withOutput(outputVolts));
    }

    @Override
    public void periodic() {
        if (isShooting) {
            final double rpm = getVelocityRPM();
            final double pidVolts = pid.calculate(rpm);

            // Clamp it to ensure it's within the +/- 12 range
            outputVolts = Math.max(-12.0, Math.min(12.0, pidVolts));
        }
    }
}
