package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.Launcher;
import com.revrobotics.spark.ClosedLoopSlot;


public class SetLauncherRPM extends Command {
  private final Launcher launcher;
  private final double rpm;
  private final ClosedLoopSlot slot;
private final boolean stopOnEnd;

  public SetLauncherRPM(Launcher launcher, double rpm) {
    this(launcher, rpm, null, true);
  }

  public SetLauncherRPM(Launcher launcher, double rpm, ClosedLoopSlot slot) {
    this(launcher, rpm, slot, true);
  }

  /**
   * @param stopOnEnd 
   */
  public SetLauncherRPM(Launcher launcher, double rpm, ClosedLoopSlot slot, boolean stopOnEnd) {
    this.launcher = launcher;
    this.rpm = rpm;
    this.slot = slot;
    this.stopOnEnd = stopOnEnd;
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    if (slot == null) launcher.setRPM(rpm);
    else launcher.setRPM(rpm, slot);
  }

  @Override
  public void execute() {
    // Closed-loop controller holds its setpoint; no need to spam setRPM().
  }

  @Override
  public void end(boolean interrupted) {
    if (stopOnEnd) launcher.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
