package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class ClimberCommand extends Command {

    private Climber m_climber;
    private boolean finished = false;


    public ClimberCommand(Climber climber) {
        this.m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setPosition(54.0);
    }

    @Override
    public void execute() {
        m_climber.setPosition(0.0);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        m_climber.setPosition(0.0);
        return finished;
    }
}
