package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDControl extends Command
{
    private double degree;
    private ShooterSubsystem shooterSubsystem;

    public ShooterPIDControl(double degree)
    {
        this.degree=degree;
        shooterSubsystem=ShooterSubsystem.getInstance();
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        shooterSubsystem.changeDegreeAim(degree);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
