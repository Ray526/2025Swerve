package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TestSwerve extends Command {

    private final Swerve swerve;

    private final Timer timer = new Timer();


    public TestSwerve (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        int step = (int)Math.floor(timer.get()) % 12;
        if (step < 2) swerve.drive(new Translation2d(0.5, 0), 0, false);
        else if (step < 4) swerve.drive(new Translation2d(-0.5, 0), 0, false);
        else if (step < 6) swerve.drive(new Translation2d(0, 0.5), 0, false);
        else if (step < 8) swerve.drive(new Translation2d(0, -0.5), 0, false);
        else if (step < 10) swerve.drive(new Translation2d(0, 0), 2, false);
        else if (step < 12) swerve.drive(new Translation2d(0, 0), -2, false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 12;
    }
}
