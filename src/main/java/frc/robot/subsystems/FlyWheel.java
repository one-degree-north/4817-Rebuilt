package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

public class FlyWheel extends SubsystemBase {
    private final String CAN_BUS = "rio";
    private final String MOTOR_TYPE = "Falcon500";
    private final double TARGET_RPM = 3000.0;
    private double targetRPM = 0.0; // initially at rest
    private TalonFX m_upper, m_lower;
    private PIDController pid = new PIDController(0, 0, 0);

    private VoltageOut voltageOut;
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.2, 0.1);

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
        targetRPM = 0.0;
        pid.reset(); // reset the PID's old error
        setControl(voltageOut.withOutput(0));
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
        targetRPM = -TARGET_RPM;
    }

    @Override
    public void periodic() {
        if (isShooting) {
            final double measuredRPM = getVelocityRPM();
            // final double measuredRPS = measuredRPM / 60.0;
            final double targetRPS = targetRPM / 60.0;

            final double ffVolts = ff.calculate(targetRPS);
            final double pidVolts = pid.calculate(measuredRPM);

            outputVolts = ffVolts + pidVolts;

            // Clamp it to ensure it's within the +/- 12 range
            outputVolts = Math.max(-12.0, Math.min(12.0, outputVolts));
            setControl(voltageOut.withOutput(outputVolts));
        }
    }
}
