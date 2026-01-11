package frc.robot.subsystems.Launcher;
// Launcher.java

import com.revrobotics.spark.SparkMax;
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

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private double targetRPM = 0.0;
  private ClosedLoopSlot activeSlot = LauncherConstants.kDefaultSlot;


  public Launcher() {

    SparkMaxConfig baseConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();


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
        .follow(leader);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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

  public void stop() {
    targetRPM = 0.0;
    leader.stopMotor(); 
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
  }

}
