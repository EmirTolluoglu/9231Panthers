
package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeSequence extends SequentialCommandGroup {
    private IntakeRollerSubsystem m_intakeRoller;
    private IntakePivotSubsystem m_intakePivot;
  public IntakeSequence() {
    m_intakeRoller=IntakeRollerSubsystem.getInstance();
    m_intakePivot=IntakePivotSubsystem.getInstance();

    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(200))),
                new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER),
                new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(10)))

    ));
  }
}
