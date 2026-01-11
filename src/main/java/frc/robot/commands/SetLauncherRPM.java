package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.Launcher;
import com.revrobotics.spark.ClosedLoopSlot;


public class SetLauncherRPM extends Command {
  private final Launcher launcher;
  private final double rpm;
  private final ClosedLoopSlot slot;

  public SetLauncherRPM(Launcher launcher, double rpm) {
    this(launcher, rpm, null);
  }

  public SetLauncherRPM(Launcher launcher, double rpm, ClosedLoopSlot slot) {
    this.launcher = launcher;
    this.rpm = rpm;
    this.slot = slot;
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    if (slot == null) launcher.setRPM(rpm);
    else launcher.setRPM(rpm, slot);
  }

  @Override
  public void execute() {
    if (slot == null) launcher.setRPM(rpm);
    else launcher.setRPM(rpm, slot);
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
