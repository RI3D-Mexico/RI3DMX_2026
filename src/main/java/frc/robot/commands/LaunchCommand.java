package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.Launcher;

public class LaunchCommand extends Command{

    private Launcher m_Launcher;
    private int state = 0;
    private double flywheelRPM, indexerDC;
    private boolean finished = false;
    private Supplier<Double> rTrigger;

    public LaunchCommand(Launcher m_Launcher, Supplier<Double> rTrigger){

        this.rTrigger = rTrigger;
        this.m_Launcher = m_Launcher;

        addRequirements(m_Launcher);


    }

    @Override
    public void initialize(){

        finished = false;

        state = 0;
        flywheelRPM = 0.0;
        indexerDC = 0.0;

    }

    @Override
    public void execute(){

        if(state == 0 ){

            flywheelRPM = 0.75;
            indexerDC = 0.0;

            if(m_Launcher.getRPM() > 1800 && rTrigger.get() > 0.2){

                state++;

            }


        }

        else if(state == 1){

            flywheelRPM = 0.75;
            indexerDC = 1.0;

            if(rTrigger.get() < 0.2){

                state++;

            }


        }

        else if(state == 2){

            finished = true;

        }

        m_Launcher.setDC(flywheelRPM);
        m_Launcher.runIndexer(indexerDC);

    }

    @Override
    public void end(boolean isInterrupted){

        state = 0;
        flywheelRPM = 0.0;
        indexerDC = 0.0;

        m_Launcher.stopAll();


    }

    @Override
    public boolean isFinished(){

        return finished;

    }
    
}
