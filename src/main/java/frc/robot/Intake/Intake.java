package frc.robot.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;

/**
 * @author 
 */
public class Intake extends Command {

    private final TalonFX motor;
    private Timer timer;

    public Intake() {
        motor = new TalonFX(44);
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        System.out.println("INITALIZING");
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        motor.set(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
        motor.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > 1;

        //break beam broken ect who cares
    }

}