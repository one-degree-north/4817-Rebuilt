package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheel extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    final private int MAX_VOLTS = 5;
    private TalonFX m_upper, m_lower;

    private VoltageOut voltageOut;
    private double voltageInput;

    public FlyWheel() {

    }
}
