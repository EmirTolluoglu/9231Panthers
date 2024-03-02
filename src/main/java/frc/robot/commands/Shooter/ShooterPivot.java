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
        addRequirements(m_shooter);
    }

    @Override
    public void initialize()
    {
        m_shooter.setPivotMotor(speed);
    }

    
}
