package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class DrivebaseSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double DRIVEBASE_RADIUS_METERS = 0.45085;
    private static final double MODULE_MAX_SPEED = 3.642; // M/s
    private static final double STATOR_CURRENT_LIMIT = 20.0; // Amps

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public DrivebaseSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();

        // Apply current limits to the motors to smooth out the accelleration and
        // braking
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
  
        // apply the limits to all 4 drive motors
        getModule(0).getDriveMotor().getConfigurator().apply(limits);
        getModule(1).getDriveMotor().getConfigurator().apply(limits);
        getModule(2).getDriveMotor().getConfigurator().apply(limits);
        getModule(3).getDriveMotor().getConfigurator().apply(limits);

    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            MODULE_MAX_SPEED,
                                            DRIVEBASE_RADIUS_METERS,
                                            new ReplanningConfig(true, true),
                                            0.004),
            this::getShouldFlipPath,
            this); // Subsystem for requirements
    }

    public boolean getShouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
}
