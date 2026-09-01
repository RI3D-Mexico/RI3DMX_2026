package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private double position;

    public IntakeCommand(IntakeSubsystem intake) {
        this.m_intake = intake;
        
        addRequirements(intake);

    }

    @Override
    public void initialize() {
        position = 0.0;
    }

    @Override
    public void execute() {
        position = 0.5;
        m_intake.setPosition(position);
        m_intake.runRoller(-0.75);
    }
    @Override
    public void  end(boolean interrupted){
        m_intake.stopRoller();
        m_intake.setPosition(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
