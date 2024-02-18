package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPivot extends Command
{
    ShooterSubsystem m_shooter;
    double speed;

    public ShooterPivot(double speed)
    {
        m_shooter=ShooterSubsystem.getInstance();
        this.speed=speed;
    }

    @Override
    public void execute()
    {
        m_shooter.setPivotMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_shooter.setPivotMotor(0);
    }
}
