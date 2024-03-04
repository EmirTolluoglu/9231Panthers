
package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakePIDControl;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeSequence extends SequentialCommandGroup {
    private IntakeSubsystem m_intake;

  public IntakeSequence() {
    m_intake=IntakeSubsystem.getInstance();
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(()->m_intake.pivotSet(Rotation2d.fromDegrees(200))),
                new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER),
                new InstantCommand(()->m_intake.pivotSet(Rotation2d.fromDegrees(10)))

    ));
  }
}
