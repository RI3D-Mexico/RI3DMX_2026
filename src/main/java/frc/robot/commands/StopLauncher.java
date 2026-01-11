package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Launcher.Launcher;

public class StopLauncher extends InstantCommand {
  public StopLauncher(Launcher launcher) {
    super(launcher::stopAll, launcher);
  }
}
