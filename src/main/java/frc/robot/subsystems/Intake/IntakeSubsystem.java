package frc.robot.subsystems.Intake;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.FeedbackSensor;
public class IntakeSubsystem extends SubsystemBase{
    private final SparkMax roller;
    private final SparkMax pivotLeader;
    private final SparkMax pivotFollower;
    private final SparkAbsoluteEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public IntakeSubsystem() {
        roller = new SparkMax(Constants.IntakeConstants.ROLLER_MOTOR_ID,MotorType.kBrushless);
        pivotLeader = new SparkMax(Constants.IntakeConstants.PIVOT_LEADER_ID,MotorType.kBrushless);
        pivotFollower = new SparkMax(Constants.IntakeConstants.PIVOT_FOLLOWER_ID,MotorType.kBrushless);

        pivotEncoder = pivotLeader.getAbsoluteEncoder();
        pivotController = pivotLeader.getClosedLoopController();

        SparkBaseConfig baseConfig = new SparkMaxConfig()
            .idleMode(Constants.IntakeConstants.kIdleMode)
            .smartCurrentLimit(Constants.IntakeConstants.kCurrentLimit)
            .openLoopRampRate(Constants.IntakeConstants.kOpenLoopRampRate)
            .closedLoopRampRate(Constants.IntakeConstants.kClosedLoopRampRate);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .apply(baseConfig)
            .inverted(Constants.IntakeConstants.kRollerInverted);

        roller.configure(
            rollerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .apply(baseConfig)
            .inverted(Constants.IntakeConstants.kPivotLeaderInverted);

        pivotConfig.absoluteEncoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360)
            .inverted(Constants.IntakeConstants.encoderInvert) 
            .zeroOffset(Constants.IntakeConstants.encoderOffset);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(
                Constants.IntakeConstants.kP_PIVOT,
                Constants.IntakeConstants.kI_PIVOT,
                Constants.IntakeConstants.kD_PIVOT
            )
            .outputRange(
                Constants.IntakeConstants.kMinOutput,
                Constants.IntakeConstants.kMaxOutput
            );
        pivotConfig.closedLoop.feedForward.sva(
            Constants.IntakeConstants.kS_PIVOT,
            Constants.IntakeConstants.kV_PIVOT,
            Constants.IntakeConstants.kA_PIVOT,
            ClosedLoopSlot.kSlot0
        );

        pivotLeader.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        SparkBaseConfig pivotFollowerConfig = new SparkMaxConfig()
                .apply(baseConfig)
                .follow(pivotLeader)
                .inverted(Constants.IntakeConstants.kPivotFollowerInverted);
        pivotFollower.configure(
            pivotFollowerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void runRoller(double speed){
        roller.set(speed);
    }
    public void stopRoller(){
        roller.stopMotor();
    }
    public void setPosition(double position){
        pivotController.setSetpoint(
            position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }
    public void stopPivot(){
        pivotLeader.stopMotor();
    }
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }

    public Command intakeRoller(){
        return run(() -> runRoller(Constants.IntakeConstants.rollerSpeed))
        .finallyDo(interrupted -> stopRoller())
        .withName("Run Intake Roller.");
    }
    public Command outtakeRoller(){
        return run(() -> runRoller(-Constants.IntakeConstants.rollerSpeed))
        .finallyDo(interrupted -> stopRoller())
        .withName("Outtake Roller.");
    }
    public Command deployIntake(){
        return runOnce(() ->setPosition(Constants.IntakeConstants.downIntakePosition)
        ).withName("Deploy Intake");
    }
    public Command retractIntake(){
        return runOnce(() ->setPosition( Constants.IntakeConstants.upIntakePosition))
        .withName("Retract Intake");
    }
    public Command movePivotTo(double position){
        return runOnce(() ->setPosition(position))
        .withName("Move Intake Position");
    }
    public Command manualPivot(double speed){
        return run(() -> pivotLeader.set(speed)
        ).finallyDo(interrupted -> stopPivot())
        .withName("Manual Pivot");
    }
    public Command stopIntake() {
        return runOnce(()->{
            stopRoller();
            stopPivot();
        }).withName("Stop all");
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot - Absolute Position",getPivotPosition());
    }
}