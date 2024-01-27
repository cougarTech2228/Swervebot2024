package frc.robot.subsystems;
/*Subsystem 'copy' */
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class ShooterSubystem2 extends SubsystemBase {
    private TalonSRX feedMotor;
    private TalonSRX flyMotor;

    private double loadSpeed = 0.2; // speed of feed motor
    private double flySpeed = 0.5; // speed of fly motor
    public ShooterSubystem2(){
        
    }

    @Override
    public void periodic(){
        super.periodic();
        
    }



}
