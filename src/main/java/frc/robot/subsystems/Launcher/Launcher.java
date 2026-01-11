package frc.robot.subsystems.Launcher;
// Launcher.java

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;


public class Launcher extends SubsystemBase {

  private final SparkMax leader =
      new SparkMax(LauncherConstants.LEADER_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

  private final SparkMax follower =
      new SparkMax(LauncherConstants.FOLLOWER_LAUNCHER_MOTOR, MotorType.kBrushless);

  private final SparkMax indexer =
      new SparkMax(LauncherConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private double targetRPM = 0.0;
  private ClosedLoopSlot activeSlot = LauncherConstants.kDefaultSlot;

  private double idleRPM = LauncherConstants.kIdleShooterRPM;
  private boolean idleEnabled = false;

  public Launcher() {

    SparkMaxConfig baseConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    baseConfig
        .idleMode(LauncherConstants.K_IDLE_MODE)
        .smartCurrentLimit(LauncherConstants.kCurrentLimit)
        .openLoopRampRate(LauncherConstants.kOpenLoopRampRate)
        .closedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
    
    leaderConfig
        .apply(baseConfig)
        .inverted(LauncherConstants.kLeaderInverted);
    
    leaderConfig.closedLoop
        .pid(LauncherConstants.kP_S0, LauncherConstants.kI_S0, LauncherConstants.kD_S0)
        .pid(LauncherConstants.kP_S1, LauncherConstants.kI_S1, LauncherConstants.kD_S1, ClosedLoopSlot.kSlot1)
        .feedForward
            .sva(LauncherConstants.kS_S0, LauncherConstants.kV_S0, LauncherConstants.kA_S0, ClosedLoopSlot.kSlot0)
            .sva(LauncherConstants.kS_S1, LauncherConstants.kV_S1, LauncherConstants.kA_S1, ClosedLoopSlot.kSlot1);

    
    leaderConfig.closedLoop.outputRange(LauncherConstants.kMinOutput, LauncherConstants.kMaxOutput);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig
        .apply(baseConfig)
        .inverted(LauncherConstants.KFollowerInverted)
        .follow(leader, true);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LauncherConstants.kIndexerCurrentLimit)
        .inverted(LauncherConstants.kIndexerInverted);
    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    encoder = leader.getEncoder();
    controller = leader.getClosedLoopController();
  }


  public void setRPM(double rpm, ClosedLoopSlot slot) {
    targetRPM = rpm;
    activeSlot = slot;
    controller.setSetpoint(rpm, ControlType.kVelocity, slot);
  }

  public void setRPM(double rpm) {
    setRPM(rpm, LauncherConstants.kDefaultSlot);
  }

  public void stopShooter() {
    targetRPM = 0.0;
    if (idleEnabled && idleRPM > 0.0) {
      setRPM(idleRPM, LauncherConstants.kDefaultSlot);
    } else {
      leader.stopMotor();
    }
  }

  public void stopAll() {
    stopIndexer();
    stopShooter();
  }

  public void stop() {
    stopAll();
  }

  public void setIdleRPM(double rpm) {
    idleRPM = rpm;
    if (idleEnabled && targetRPM <= 0.0) {
      setRPM(idleRPM, LauncherConstants.kDefaultSlot);
    }
  }

  public void enableIdle(boolean enable) {
    idleEnabled = enable;
    if (idleEnabled && targetRPM <= 0.0 && idleRPM > 0.0) {
      setRPM(idleRPM, LauncherConstants.kDefaultSlot);
    }
    if (!idleEnabled && targetRPM <= 0.0) {
      leader.stopMotor();
    }
  }

  public void runIndexer(double percent) {
    indexer.set(percent);
  }

  public void stopIndexer() {
    indexer.stopMotor();
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public boolean atSpeed() {
    return targetRPM > 0.0
        && Math.abs(getRPM() - targetRPM) <= LauncherConstants.kRpmTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/TargetRPM", targetRPM);
    SmartDashboard.putNumber("Launcher/ActualRPM", getRPM());
    SmartDashboard.putBoolean("Launcher/AtSpeed", atSpeed());
    SmartDashboard.putString("Launcher/Slot", activeSlot.toString());

    SmartDashboard.putBoolean("Launcher/IdleEnabled", idleEnabled);
    SmartDashboard.putNumber("Launcher/IdleRPM", idleRPM);
  }

}
