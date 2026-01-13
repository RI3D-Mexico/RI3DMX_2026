package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher.Launcher;

public class TeleopLauncherControl extends Command {
  private final Launcher launcher;
  private final DoubleSupplier triggerAxis;

  private final double idleRPM;
  private final double shootRPM;
  private final double indexerPercent;

  public TeleopLauncherControl(Launcher launcher, DoubleSupplier triggerAxis) {
    this(
        launcher,
        triggerAxis,
        LauncherConstants.kIdleShooterRPM,
        LauncherConstants.kDefaultShooterRPM,
        LauncherConstants.kIndexerFeedPercent);
  }

  public TeleopLauncherControl(
      Launcher launcher,
      DoubleSupplier triggerAxis,
      double idleRPM,
      double shootRPM,
      double indexerPercent) {
    this.launcher = launcher;
    this.triggerAxis = triggerAxis;
    this.idleRPM = idleRPM;
    this.shootRPM = shootRPM;
    this.indexerPercent = indexerPercent;
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    launcher.setIdleRPM(idleRPM);
    launcher.enableIdle(true);
    launcher.setRPM(idleRPM);
    launcher.stopIndexer();
  }

  @Override
  public void execute() {
    final double trig = triggerAxis.getAsDouble();
    final boolean wantsShoot = trig > LauncherConstants.kShootTriggerDeadband;

    if (wantsShoot) {
      launcher.setRPM(shootRPM);

      boolean canFeed = true;
      if (LauncherConstants.kOnlyFeedWhenAtSpeed) {
        canFeed = launcher.atSpeed();
      }

      if (canFeed) {
        launcher.runIndexer(indexerPercent);
      } else {
        launcher.stopIndexer();
      }
    } else {
      launcher.setRPM(idleRPM);
      launcher.stopIndexer();
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stopIndexer();
    launcher.setRPM(idleRPM);
    launcher.enableIdle(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
