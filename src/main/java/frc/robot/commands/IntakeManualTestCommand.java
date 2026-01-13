package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeManualTestCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier rollerButtonIn;
    private final BooleanSupplier rollerButtonOut;
    private final IntSupplier pOVSupplier;

    private final double stowRot;
    private final double intakeDownRot;

    private int lastPov = 1;

    public IntakeManualTestCommand( IntakeSubsystem m_intake,BooleanSupplier rollerButtonIn,BooleanSupplier rollerButtonOut, IntSupplier pOVSupplier,double stowRot,double intakeDownRot){
        this.m_intake = m_intake;
        this.rollerButtonIn = rollerButtonIn;
         this.rollerButtonOut = rollerButtonOut;
        this.pOVSupplier = pOVSupplier;
        this.stowRot = stowRot;
        this.intakeDownRot = intakeDownRot;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        lastPov = -1;
        m_intake.stopRoller();
    }

    @Override
    public void execute() {
        if (rollerButtonIn.getAsBoolean()){
            m_intake.runRoller(-0.75);
        }else if(rollerButtonOut.getAsBoolean()){
            m_intake.runRoller(0.75);
        }else{
            m_intake.stopRoller();
        }

        int pov = pOVSupplier.getAsInt();
        if(pov != lastPov){
            m_intake.setPivotPosition(intakeDownRot);
        }else if(pov== 180){
            m_intake.setPivotPosition(stowRot);
        }
        lastPov =  pov;
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
