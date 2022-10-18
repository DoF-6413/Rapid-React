package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/** Shoots High after positioning using limelight */
public class LLAimAndShoot extends SequentialCommandGroup {
  /** Creates a new LimelightShoot. */
  public LLAimAndShoot(LimelightSubsystem light, IndexerSubsystem index, ShooterSubsystem shoot, DrivetrainSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LimelightPrep(index,shoot),
      //Aims Limelight to Ideal Shoot Position
      //Note: Should ramp up shooter and move indexer back so it does not accidentally shoot
      new LimelightAim(drive, light),
      //move indexer forward and disable
      new LimelightShoot(index, shoot)

    );
  }
}