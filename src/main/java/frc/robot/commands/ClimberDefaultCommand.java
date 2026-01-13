package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class ClimberDefaultCommand extends Command {

    private Climber m_Climber;

    public ClimberDefaultCommand(Climber m_Climber){

        this.m_Climber = m_Climber;

        addRequirements(m_Climber);

    }

    @Override
    public void execute(){

        m_Climber.setPosition(0.1);

    }
    
}
