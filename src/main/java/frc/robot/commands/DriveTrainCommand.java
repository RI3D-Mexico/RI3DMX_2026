package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveTrain;

public class DriveTrainCommand extends Command{

    private DriveTrain driveTrain;
    private Supplier<Double> translation, strafe, rotation;
    private Supplier<Boolean> changeDrive, resetGyro;
    private boolean isRobotCentric = false;

    
    private SlewRateLimiter translationLimit = new SlewRateLimiter(2.0);
    private SlewRateLimiter strafeLimit = new SlewRateLimiter(2.0);
    private SlewRateLimiter rotationLimit = new SlewRateLimiter(2.0);

    

    public DriveTrainCommand(DriveTrain m_DriveTrain, Supplier<Double> translation, Supplier<Double> strafe, Supplier<Double> rotation, Supplier<Boolean> changeDrive, Supplier<Boolean> resetGyro){

        this.driveTrain = m_DriveTrain;
        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.changeDrive = changeDrive;
        this.resetGyro = resetGyro;
        isRobotCentric = true;

        addRequirements(m_DriveTrain);

    }
    
    @Override
    public void execute() {

        if(changeDrive.get()){
            
            isRobotCentric = !isRobotCentric;

        }    

        if(resetGyro.get()){

            driveTrain.zeroGyro();

        }    

        double translationVal = translationLimit.calculate(MathUtil.applyDeadband(translation.get(), 
                                                                                  Constants.Swerve.stickDeadband));
        double strafeVal = strafeLimit.calculate(MathUtil.applyDeadband(strafe.get(), 
                                                                        Constants.Swerve.stickDeadband));
        double rotationVal = rotationLimit.calculate(MathUtil.applyDeadband(rotation.get(), 
                                                                            Constants.Swerve.stickDeadband));
                                                                            
        SmartDashboard.putString("Drive Mode:",  (isRobotCentric ? "Driving Field Oriented"
                                                                     : "Driving Robot Oriented"));

        driveTrain.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, isRobotCentric, true);
    }
}