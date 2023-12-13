package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private SwerveRequest.RobotCentric robotCentric = new RobotCentric().withIsOpenLoop(true);

    private PathConstraints pathConstraints = new PathConstraints(
        5.0, 4.5,
        Units.degreesToRadians(540), Units.degreesToRadians(720)
    );

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configPathFollowing();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configPathFollowing();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume  */
        updateSimState(0.02, 12);
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        var states = getState().ModuleStates;
        return ChassisSpeeds.fromFieldRelativeSpeeds(m_kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]), m_pigeon2.getRotation2d());
    }

    public void driveRobotRelativeSpeeds(ChassisSpeeds speeds) {
        this.setControl(robotCentric
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    public void configPathFollowing() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::seedFieldRelative,
            this::getFieldRelativeSpeeds,
            this::driveRobotRelativeSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), 
                new PIDConstants(5.0, 0, 0), 
                4.5,
                0.6,
                new ReplanningConfig(true, true)
            ),
            this
        );
    }

    public Command goToPoint(Pose2d target, double meterRotationDelay) {
        return AutoBuilder.pathfindToPose(target, pathConstraints, 0, meterRotationDelay);
    }

    public Command goToDoubleWithEntry(double meterRotationDelay) {
        PathPlannerPath entryPath = PathPlannerPath.fromPathFile("ToNode2");
        return AutoBuilder.pathfindThenFollowPath(entryPath, pathConstraints, meterRotationDelay);
    }
}
