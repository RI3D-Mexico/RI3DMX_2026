package frc.robot.commands;

import frc.robot.subsystems.Climber.Climber;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualClimb extends Command {
  private final Climber climber;
  private final DoubleSupplier percentSupplier;

  public ManualClimb(Climber climber, DoubleSupplier percentSupplier) {
    this.climber = climber;
    this.percentSupplier = percentSupplier;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.setManual(percentSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
