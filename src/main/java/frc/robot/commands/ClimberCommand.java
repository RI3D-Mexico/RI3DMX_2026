package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class ClimberCommand extends Command {

    private Climber m_climber;


    public ClimberCommand(Climber m_Climber) {
        this.m_climber = m_Climber;

        addRequirements(m_Climber);
    }

    @Override
    public void initialize() {

        m_climber.setPosition(3.5);
    }

    @Override
    public void execute() {

        System.out.println("AQUI PT2");

        m_climber.setPosition(3.5);
    }

    @Override
    public void end(boolean isInterrupted){

        m_climber.setPosition(0.1);


    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}
