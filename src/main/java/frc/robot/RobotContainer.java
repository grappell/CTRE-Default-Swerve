// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.Intake;
import frc.robot.generated.TunerConstants;
import java.util.Map;
import java.util.HashMap;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);//.withDeadband(0.5).withRotationalDeadband(1); // I want field-centric driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  

  Telemetry logger = new Telemetry(MaxSpeed);

  SendableChooser<Command> autoChooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(deadBand(-joystick.getLeftY(), 0.15) * MaxSpeed * (joystick.leftBumper().getAsBoolean() ? 0.25 : 1)) // Drive forward with negative Y (forward)
            .withVelocityY(deadBand(-joystick.getLeftX(), 0.15) * MaxSpeed * (joystick.leftBumper().getAsBoolean() ? 0.25 : 1)) // Drive left with negative X (left)
            .withRotationalRate(deadBand(-joystick.getRightX(), 0.2) * MaxAngularRate * (joystick.leftBumper().getAsBoolean() ? 0.5 : 1)) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.y().onTrue(Commands.runOnce(() -> drivetrain.tareEverything()));
    joystick.b().whileTrue(new Intake());

    drivetrain.registerTelemetry(logger::telemeterize);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    joystick.povDown().onTrue(drivetrain.goToDoubleWithEntry(0));
    joystick.povUp().onTrue(drivetrain.goToPoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 0.2));
  }

  public static double deadBand(double val, double deadband){
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public RobotContainer() {
    NamedCommands.registerCommand("IntakeNote", new Intake());
    drivetrain.configPathFollowing();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
