package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

public class FlyWheel extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    final private double SHOOT_VOLTS = 5.0;
    private TalonFX m_upper, m_lower;

    private VoltageOut voltageOut;
    private double voltageInput;

    public FlyWheel() {
        this.m_lower = new TalonFX(1, CAN_BUS);
        this.m_upper = new TalonFX(2, CAN_BUS);
        this.voltageInput = 0.5;
        this.voltageOut = new VoltageOut(0);
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
     * Get the state of a motor (either clockwise, counterclockwise, or at rest) as
     * an int
     * 
     * @param motor
     *              - the motor you are getting the state of
     * @return the state of the motor: -1 = clockwise, 1 = counterclockwise, 0 = at
     *         rest
     */
    public int getMotorState(TalonFX motor) {
        double velocity = motor.getVelocity().getValueAsDouble();

        // Use 0.01 to avoid noise issue
        if (velocity > 0.01) {
            // Clockwise
            return -1;
        } else if (velocity < -0.01) {
            // Counterclockwise
            return 1;
        }

        // At rest
        return 0;
    }

    /**
     * Increase the voltage of a specific motor
     * 
     * @param motor
     *              - the motor you are trying to increase the voltage
     * @param input
     *              - the amount of voltage you are trying to increase (this is
     *              always positive)
     */
    public void voltageUp(TalonFX motor, double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(motor, voltageOut.withOutput(input));
    }

    /**
     * 
     * Increase the upper motor's voltage with the defined `voltageInput` variable
     */
    public void voltageUp() {
        setControl(m_upper, voltageOut.withOutput(voltageInput));
    }

    /**
     * Decrease the voltage of a specific motor
     * 
     * @param motor
     *              - the motor you are trying to decrease the voltage
     * @param input
     *              - the amount of voltage you are trying to decrease (this is
     *              always positive)
     */
    public void voltageDown(TalonFX motor, double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(motor, voltageOut.withOutput(-input));
    }

    /**
     * Decrease the upper motor's voltage with the defined `voltageInput` variable
     */
    public void voltageDown() {
        setControl(m_upper, voltageOut.withOutput(-voltageInput));
    }

    /**
     * Stop the 2 motors
     */
    public void stopMotor() {
        setControl(m_upper, voltageOut.withOutput(0));
    }

    /**
     * Get the velocity of a specified motor
     * 
     * @param motor
     *              - the motor you are getting the velocity
     * @return the velocity of that motor
     */
    public double getVelocity(TalonFX motor) {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the velocity of the upper motor
     * 
     * @return the velocity of the upper motor
     */
    public double getVelocity() {
        return m_upper.getVelocity().getValueAsDouble();
    }

    public void run() {
        // Positive = Clockwise, Negative = Counter Clockwise
        setControl(m_upper, voltageOut.withOutput(-SHOOT_VOLTS));
    }
}
