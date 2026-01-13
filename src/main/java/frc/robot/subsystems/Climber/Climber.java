package frc.robot.subsystems.Climber;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {

  private final SparkMax climberMotor =
      new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;


  private double holdSetpointRot = 0.0;

  public Climber() {

    
    encoder = climberMotor.getEncoder();
    controller = climberMotor.getClosedLoopController();

    SparkMaxConfig climberConfig = new SparkMaxConfig();

    climberConfig
        .idleMode(K_IDLE_MODE)
        .smartCurrentLimit(kCurrentLimit)
        .openLoopRampRate(kOpenLoopRampRate)
        .closedLoopRampRate(kClosedLoopRampRate);


    climberConfig.inverted(kInverted);

    climberMotor.configure(
        climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    holdSetpointRot = getPositionRot();
  }

  public double getPositionRot() {
    return encoder.getPosition() / kGearRatio;
  }

  public void zeroPosition() {
    encoder.setPosition(0.0);
    holdSetpointRot = 0.0;
  }

  public void setPosition(double holdSetpointRot ) {
    controller.setSetpoint(holdSetpointRot, ControlType.kPosition);
  }

  public void setManual(double percent) {
    percent = MathUtil.applyDeadband(percent, kDeadband);

    double pos = getPositionRot();

    if (pos <= kMinPosRot && percent < 0) percent = 0.0;
    if (pos >= kMaxPosRot && percent > 0) percent = 0.0;

    if (percent > 0) percent *= kMaxUpPercent;
    if (percent < 0) percent *= kMaxDownPercent;

    if (Math.abs(percent) < 1e-4) {
      if (kHoldWhenIdle) {
        holdPosition();
      } else {
        stop();
      }
      return;
    }

    holdSetpointRot = pos;
    climberMotor.set(percent);
  }

  public void holdPosition() {
    double pos = getPositionRot();
    double error = holdSetpointRot - pos;

    double cmd = MathUtil.clamp(kHold_kP * error, -kHoldMaxPercent, kHoldMaxPercent);

    if (pos <= kMinPosRot && cmd < 0) cmd = 0.0;
    if (pos >= kMaxPosRot && cmd > 0) cmd = 0.0;

    climberMotor.set(cmd);
  }

  public void stop() {
    climberMotor.stopMotor();
  }
}
