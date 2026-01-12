package frc.robot.subsystems.Intake;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    //Motor controllers for the roller and pivot mechanisms
    private SparkMax roller;
    private SparkMax pivotLeader;
    private SparkMax pivotFollower;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;


    public IntakeSubsystem() {
        roller = new SparkMax(Constants.IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        pivotLeader = new SparkMax(Constants.IntakeConstants.PIVOT_LEADER_ID, MotorType.kBrushless);
        pivotFollower = new SparkMax(Constants.IntakeConstants.PIVOT_FOLLOWER_ID, MotorType.kBrushless);

        pivotEncoder = pivotLeader.getEncoder();
        pivotController = pivotLeader.getClosedLoopController();

        //base configuration
        SparkBaseConfig baseConfig = new SparkMaxConfig()
            .idleMode(Constants.IntakeConstants.kIdleMode)
            .smartCurrentLimit(Constants.IntakeConstants.kCurrentLimit)
            .openLoopRampRate(Constants.IntakeConstants.kOpenLoopRampRate)
            .closedLoopRampRate(Constants.IntakeConstants.kClosedLoopRampRate);

        //configure motor roller
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
        .apply(baseConfig)
        .inverted(Constants.IntakeConstants.kRollerInverted);
        
        roller
        .configure(rollerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Configuration for pivot leader motor with PIDF
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
        .apply(baseConfig)
        .inverted(Constants.IntakeConstants.kPivotLeaderInverted);

        pivotConfig.closedLoop.pid(
            Constants.IntakeConstants.kP_PIVOT,
        Constants.IntakeConstants.kI_PIVOT,
        Constants.IntakeConstants.kD_PIVOT);

        pivotConfig.closedLoop.feedForward.sva(
            Constants.IntakeConstants.kS_PIVOT, 
            Constants.IntakeConstants.kV_PIVOT,
            Constants.IntakeConstants.kA_PIVOT,
            ClosedLoopSlot.kSlot0);

            pivotConfig.closedLoop.outputRange(
                Constants.IntakeConstants.kMinOutput,
            Constants.IntakeConstants.kMaxOutput);

            pivotLeader.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            //make the second motor become a follower
            SparkBaseConfig pivotFollowerConfig = new SparkMaxConfig().apply(baseConfig).follow(pivotLeader)
            .inverted(Constants.IntakeConstants.kPivotFollowerInverted);
            pivotFollower.configure(pivotFollowerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     
    }

    public void runRoller() {
        roller.set(0.75);
    }

    public void stopRoller() {
        roller.set(0);
    }

    public void setPivotPosition(double angleRotations) {
        pivotController.setSetpoint(angleRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stopPivot() {
        pivotLeader.stopMotor();
    }
    
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/Position",getPivotPosition());
    }
}
/*
 * TODO: 
 * Add absolute encoder
 * Configure absolute encoder
 * nice to have: feedforward
 * create intake comands
 */