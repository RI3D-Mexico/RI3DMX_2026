package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;

    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.runRoller();
    }

    @Override
    public void  end(boolean interrupted){
        m_intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
