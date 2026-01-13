package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private double position;
    private boolean finished;
    private BooleanSupplier Xbutton;

    public IntakeCommand(IntakeSubsystem intake,BooleanSupplier Xbutton) {
        this.m_intake = intake;
        this.Xbutton = Xbutton;
        
        addRequirements(intake);

    }

    @Override
    public void initialize() {
        position = 0.0;
        finished = false;
    }

     @Override
    public void execute() {
        position = 0.5;
        m_intake.setPivotPosition(position);
        m_intake.runRoller();
    }
    @Override
    public void  end(boolean interrupted){
        m_intake.stopRoller();
        m_intake.setPivotPosition(0.0)
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
