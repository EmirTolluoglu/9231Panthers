
package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakePIDControl;
import frc.robot.commands.Intake.IntakeRoller;


public class IntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  public IntakeSequence() {

    addCommands(
      new SequentialCommandGroup(
        new IntakePIDControl(1.353), //out
        new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER),
        new IntakePIDControl(1.04) // top
    ));
  }
}
