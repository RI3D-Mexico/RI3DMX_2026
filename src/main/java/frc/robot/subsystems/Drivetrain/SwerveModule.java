package frc.robot.subsystems.Drivetrain;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.ModuleOptimizer;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    
    public int moduleID;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private Rotation2d encoderOffset;

    private SparkMax driveMotor;
    private SparkMax steerMotor;

    private RelativeEncoder steerMotorEncoder;
    private RelativeEncoder driveMotorEncoder;
    private CANcoder angleEncoder;

    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    private SwerveModuleState targetState = new SwerveModuleState();

    private final SimpleMotorFeedforward feedforward =  new SimpleMotorFeedforward(Constants.Swerve.driveKS, 
                                                                                   Constants.Swerve.driveKV, 
                                                                                   Constants.Swerve.driveKA);    
    
    public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants){
        
        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;
        encoderOffset = moduleConstants.encoderOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();
        angleEncoder.getAbsolutePosition().waitForUpdate(0.2);

        steerMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        steerMotorEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getClosedLoopController();
        configSteerMotor();

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotorEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = ModuleOptimizer.optimize(desiredState, getState().angle);

        this.targetState = desiredState;
    
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }
    private void resetToAbsolute() {

        double absolutePosition = getCANCoderAngle().getDegrees() - angleOffset.getDegrees();

        steerMotorEncoder.setPosition(absolutePosition);

    }

    private void configSteerMotor() {

        SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
        ClosedLoopConfig steerMotorCLConfig = new ClosedLoopConfig();
        EncoderConfig steerMotorEncoderConfig = new EncoderConfig();
        

        steerMotorEncoderConfig.positionConversionFactor(Constants.Swerve.angleConversionFactor);
        
        steerMotorCLConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0.08).i(0.0).d(0.0)
                          .positionWrappingEnabled(true);

        steerMotorConfig.idleMode(IdleMode.kBrake)
                        .inverted(Constants.Swerve.angleInvert)
                        .voltageCompensation(Constants.Swerve.voltageComp)
                        .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
                        .apply(steerMotorCLConfig)
                        .apply(steerMotorEncoderConfig);

        steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetToAbsolute();

    }

    private void configDriveMotor() {

        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        ClosedLoopConfig driveMotorCLConfig = new ClosedLoopConfig();
        EncoderConfig driveMotorEncoderConfig = new EncoderConfig();

        driveMotorEncoderConfig.positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);

        driveMotorCLConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0.2).i(0.0).d(0.0)
                          .positionWrappingEnabled(true);
        
        driveMotorConfig.idleMode(IdleMode.kBrake)
                        .inverted(Constants.Swerve.angleInvert)
                        .voltageCompensation(Constants.Swerve.voltageComp)
                        .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
                        .apply(driveMotorCLConfig)
                        .apply(driveMotorEncoderConfig);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveMotorEncoder.setPosition(0.0);

        
    }

    
    private void configAngleEncoder() {

        var angleEncoderConfigs = new CANcoderConfiguration();
        angleEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        angleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

     
        angleEncoder.getConfigurator().apply(angleEncoderConfigs);

    }
    

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {

            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(power);
        
        } 
        else {

            driveController.setSetpoint(feedforward.calculate(desiredState.speedMetersPerSecond), 
                                         ControlType.kVelocity, 
                                         ClosedLoopSlot.kSlot0);
              
        }
    }

    private void setAngle(SwerveModuleState desiredState) {

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        steerController.setSetpoint(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;

    }

    public double getDrivePosition(){

        return driveMotorEncoder.getPosition();

    }


    public Rotation2d getSteerAngle() {

        if (RobotBase.isSimulation()) {
            return targetState.angle;
        }
        // ... existing logic for real robot ...
        double angle = steerMotorEncoder.getPosition();
        return Rotation2d.fromDegrees(angle);

    }

    public Rotation2d getCANCoderAngle() {

        double angle = ((angleEncoder.getAbsolutePosition().getValueAsDouble()) * 360.0) - encoderOffset.getDegrees();
        return Rotation2d.fromDegrees(angle);

    }

    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(driveMotorEncoder.getPosition(), getSteerAngle());

    }

    public SwerveModuleState getState() {

        double velocity;
        
        if (RobotBase.isSimulation()) {
            // IN SIMULATION:
            // The encoder won't report velocity automatically unless we do complex sim setup.
            // So, we just return the speed we *wanted* the module to have.
            velocity = targetState.speedMetersPerSecond;
        } else {
            // REAL ROBOT:
            // Read the actual velocity from the Spark Max
            velocity = driveMotorEncoder.getVelocity();
        }

        return new SwerveModuleState(velocity, getSteerAngle());

    }

    public SwerveModuleState getStateEncoder() {

        double velocity;
        
        if (RobotBase.isSimulation()) {
            // IN SIMULATION:
            // The encoder won't report velocity automatically unless we do complex sim setup.
            // So, we just return the speed we *wanted* the module to have.
            velocity = targetState.speedMetersPerSecond;
        } else {
            // REAL ROBOT:
            // Read the actual velocity from the Spark Max
            velocity = driveMotorEncoder.getVelocity();
        }

        return new SwerveModuleState(velocity, getSteerAngle());

    }

    /**
     * Updates the simulated state of the module.
     * @param dtSeconds The time elapsed since the last loop (usually 0.02s)
     */
    public void simulationPeriodic(double dtSeconds) {
        // --- 1. Simulate Drive Motor (NEO) ---
        // Calculate the distance the wheel would have traveled in this timestep
        // Distance = Velocity * Time
        double driveDeltaMeters = targetState.speedMetersPerSecond * dtSeconds;
        
        // Get the current simulated position
        double currentDrivePosition = driveMotorEncoder.getPosition();
        
        // Update the SparkMax internal sim state
        // NOTE: setPosition acts on the "converted" value if you configured a conversion factor.
        driveMotorEncoder.setPosition(currentDrivePosition + driveDeltaMeters);


        // --- 2. Simulate Steer Motor (NEO) ---
        // For simplicity in simulation, we assume the steer motor snaps instantly to the target angle.
        // If your robot spins wildly in sim, you might need to divide by your conversion factor here
        // e.g. setPosition(targetState.angle.getDegrees() / Constants.Swerve.angleConversionFactor);
        steerMotorEncoder.setPosition(targetState.angle.getDegrees());

        // --- 3. Simulate CANcoder (Phoenix 6) ---
        // Phoenix 6 devices have a specific "SimState" object we must write to.
        // CANcoders expect units in Rotations.
        angleEncoder.getSimState().setRawPosition(targetState.angle.getRotations());
    }

    @Override
    public void periodic(){

    }
}