package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCmd extends Command
{
    ClimberSubsystem m_climber;

    double speed;
    public ClimberCmd(double speed)
    {
        m_climber=ClimberSubsystem.getInstance();
        this.speed=speed;
        addRequirements(m_climber);
    }

    @Override
    public void initialize()
    {
        m_climber.climberMotors(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_climber.climberMotors(0);
    }
}