
package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterRollerSubsystem;



public class ShootSequence extends SequentialCommandGroup {
    private IntakePivotSubsystem m_intake;
    private IntakeRollerSubsystem m_intakeRoller;
    private ShooterRollerSubsystem m_shooterRoller;
    private ShooterPivotSubsystem m_shooterPivot;
  public ShootSequence() {
        m_intake=IntakePivotSubsystem.getInstance();
        m_intakeRoller=IntakeRollerSubsystem.getInstance();
        m_shooterRoller=ShooterRollerSubsystem.getInstance();
        m_shooterPivot=ShooterPivotSubsystem.getInstance();


    addCommands(
        
      new ParallelCommandGroup(

        
        new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER),
        new SequentialCommandGroup(
            new WaitCommand(2),
            new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER))
        )
      ));
  }
}
