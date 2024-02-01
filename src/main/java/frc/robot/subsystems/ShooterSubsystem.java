package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonSRX mShooterFeedMotor;
    private TalonSRX mShooterFlywheelMotor;

    private final static double LOAD_SPEED = -0.4;
    public ShooterSubsystem() {
        mShooterFeedMotor = new TalonSRX(TunerConstants.kShooterFeedMotorId);
        mShooterFlywheelMotor = new TalonSRX(TunerConstants.kShooterFlywheelMotorId);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public void test() {
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 1);
        mShooterFeedMotor.set(ControlMode.PercentOutput, 1);
    }

    public void startFlywheel() {
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 1);
    }

    public void feedNote() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, 1);
    }

    public void stopMotors() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, 0);
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, 0);
    }

    public void loadNote() {
        mShooterFeedMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
        mShooterFlywheelMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    }
};