package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRoller extends Command 
{
    private IntakeSubsystem intakeSubsystem;
    private double speed;

    public IntakeRoller(double speed)
    {
        intakeSubsystem=IntakeSubsystem.getInstance();
        this.speed=speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        intakeSubsystem.setRollerMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        intakeSubsystem.setRollerMotor(0);
    }
}