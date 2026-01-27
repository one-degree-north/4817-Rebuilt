package frc.robot.subsystems;

//Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;

// test commit

public class Storage extends SubsystemBase {
    //PID Constants and Feedforward
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KS = 0.1;
    private final double KV = 0.2;

    private final String CAN_BUS = "rio";
    private final String MOTOR_TYPE = "Falcon500";
    private final double TOLERANCE = 0.1;
    private double targetRPS = 0.0 ; //positive = counterclockwise, negative = clockwise, initially at rest

    private TalonFX m_left, m_right;
    @SuppressWarnings("unused")
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Storage() {
        this.m_left = new TalonFX(3, CAN_BUS);
        this.m_right = new TalonFX(4, CAN_BUS);
        ConfigureMotors();
    
    }

    private void ConfigureMotors(){
        var motorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
        .withMotorOutput(
            MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Brake,InvertedValue.CounterClockwise_Positive))
        .withFeedback(MotorConfigs.getFeedbackConfigs(1/1));


    var slot0Configs = motorConfigs.Slot0;
        slot0Configs.kP = KP; // Proportional gain 
        slot0Configs.kI = KI; // Integral gain 
        slot0Configs.kD = KD; // Derivative gain
        slot0Configs.kS = KS; // Static feedforward in volts
        slot0Configs.kV = KV; // Velocity feedforward (Volts / RPS)

    m_left.getConfigurator().apply(motorConfigs);
    m_right.getConfigurator().apply(motorConfigs);
    m_right.setControl(new Follower(m_left.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // Getting velocity of the motors (rev/sec)
    private double getVelocityRPS() {
    return m_left.getVelocity().getValueAsDouble();

    }

    @SuppressWarnings("unused")
    private void setControl(ControlRequest req) {
        if (m_left.isAlive()) {
            m_left.setControl(req);
        }
    }

     private boolean atSetpoint() {
        return Math.abs(getVelocityRPS() - targetRPS) <= TOLERANCE;
    }
    
 public boolean atSetpoint(boolean withTolerance) {
        if (withTolerance)
            return atSetpoint();
        return Double.compare(getVelocityRPS(), targetRPS) == 0;
    }

    public void setTargetRPS(double RPS){
        targetRPS = RPS;
    }

    //deal with this code later, google what needs to be in a piece of code before you push it




   


}
