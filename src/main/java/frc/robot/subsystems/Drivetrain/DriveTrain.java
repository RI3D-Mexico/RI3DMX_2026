package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{

    private Pigeon2 gyro;


    private SwerveModule[] swerveModules;
    private SwerveModulePosition[] swervePositions;

    private Field2d field;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    
    private final StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve/ActualStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> publisherTargetStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve/TargetStates", SwerveModuleState.struct).publish();

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Pose", Pose2d.struct).publish();

    public DriveTrain(){
        gyro = new Pigeon2(0);

        

        field = new Field2d();
        SmartDashboard.putData("Field: ", field);
        
        
        
        swerveModules =
            new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants),
            };

        swervePositions = 
            new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            };
            
        // swerveWheels =
        //     new SwerveDriveWheelPositions[]{
        //         new SwerveDriveWheelPositions(swervePositions)
        //     };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveOdoKinematics, 
                                                 getOdoYaw(), 
                                                 swervePositions); 
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveOdoKinematics, 
                                                           getOdoYaw(), 
                                                           swervePositions, 
                                                           getPose());
        field.setRobotPose(getPose());

        configureGyro();
        configureAutoBuilder();
    }

     private void configureAutoBuilder() {
        try {
            System.out.println("Configuring Auto");
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                this::getRobotRelativeSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void configureGyro(){

        var pigeonConfig = new Pigeon2Configuration();

        pigeonConfig.withMountPose(new MountPoseConfigs().withMountPoseYaw(90.0));
        zeroGyro();
        

    }

    public void drive(Translation2d translation, double rotation, boolean isFieldDrive, boolean isOpenloop){

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveOdoKinematics.toSwerveModuleStates(isFieldDrive ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), 
                                                                                                                        translation.getY(),
                                                                                                                        rotation, 
                                                                                                                        getYaw())
                                                                                    : new ChassisSpeeds(-translation.getY(),
                                                                                                        translation.getX(),
                                                                                                        rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule module : swerveModules){

            module.setDesiredState(swerveModuleStates[module.moduleID], isOpenloop);

        }

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    
        for (SwerveModule module : swerveModules) {

          module.setDesiredState(desiredStates[module.moduleID], false);

        }

      }

      public Rotation2d getYaw() {

        double angle = (Constants.Swerve.invNavX) ? 360 - gyro.getYaw().getValueAsDouble() //gyro is flipped
                                                  : gyro.getYaw().getValueAsDouble();

        return Rotation2d.fromDegrees(angle);

    }

    public Rotation2d getOdoYaw() {

        double angle = (!Constants.Swerve.invNavX) ? 360 - gyro.getYaw().getValueAsDouble() //gyro is flipped
                                                   : gyro.getYaw().getValueAsDouble();
        return Rotation2d.fromDegrees(angle);

    }

    public void zeroGyro() {
    gyro.setYaw(0.0);
    
    Pose2d currentPose = swervePoseEstimator.getEstimatedPosition();

    Pose2d newPose = new Pose2d(currentPose.getTranslation(), new Rotation2d(0.0));
    
    swervePoseEstimator.resetPosition(getOdoYaw(), getPositions(), newPose);
    swerveOdometry.resetPosition(getOdoYaw(), getPositions(), newPose);
    
    System.out.println("Gyro and Pose Zeroed!");
}

    public SwerveModuleState[] getTargetStates() {
        return new SwerveModuleState[] {
            swerveModules[0].getTargetState(), // You might need to add this method to your Module class
            swerveModules[1].getTargetState(),
            swerveModules[2].getTargetState(),
            swerveModules[3].getTargetState()
        };
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule module : swerveModules) {

          states[module.moduleID] = module.getState();

        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule module : swerveModules) {

          positions[module.moduleID] = module.getPosition();

        }

        return positions;
    }

    public Pose2d getPose() {

        return swerveOdometry.getPoseMeters();
        //return swervePoseEstimator.getEstimatedPosition();
    }


    public void resetPose(Pose2d pose) {
    
        swerveOdometry.resetPosition(getOdoYaw(), getPositions(), pose);
    
    }

    public void driveRobotRelative(ChassisSpeeds speeds){

        this.drive(new Translation2d(speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond ),speeds.omegaRadiansPerSecond,false,true);
    
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        
        return Constants.Swerve.swerveOdoKinematics.toChassisSpeeds(getStates());

    }
    
    public void updateOdometry() {

        

        swervePoseEstimator.update(getOdoYaw(),
                                   getPositions());
        
        swerveOdometry.resetPosition(getOdoYaw(), getPositions(), swervePoseEstimator.getEstimatedPosition());
    }

    @Override
    public void simulationPeriodic() {
    
    ChassisSpeeds speeds = Constants.Swerve.swerveOdoKinematics.toChassisSpeeds(getTargetStates());

    
    double changeInYawDegrees = Math.toDegrees(speeds.omegaRadiansPerSecond * 0.01);
    
    // Update the simulated Pigeon2 so your field-centric drive works
    gyro.getSimState().addYaw(changeInYawDegrees);

    Pose2d currentPose = swervePoseEstimator.getEstimatedPosition();
    
    // Apply the movement (Twist) to the current pose
    Pose2d newPose = currentPose.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * 0.02, 
            speeds.vyMetersPerSecond * 0.02, 
            speeds.omegaRadiansPerSecond * 0.02
        )
    );

    // Force the pose estimator to this new calculated position
    // (We use resetPosition because the real encoders are still reading 0)
    swervePoseEstimator.resetPosition(
        getOdoYaw(), 
        getPositions(), // These are still 0, but it doesn't matter because we overwrite the pose below
        newPose
    );
}

    @Override
    public void periodic() {
        //swerveOdometry.update(getOdoYaw(), getPositions());
        //swervePoseEstimator.update(getOdoYaw(), getPositions());
        // //swervePoseEstimator.addVisionMeasurement(getPose(), 0);
        // limef = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limef);
        // limeb = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limeb);        
        
        
        SmartDashboard.putString("Odometry: ", swerveOdometry.getPoseMeters().getTranslation().toString());
        SmartDashboard.putNumber("gyro", getYaw().getDegrees());
        SmartDashboard.putNumber("gyro Odo", getOdoYaw().getDegrees());

        // if(limeb.getEntry("tv").getDouble(0) == 1){

        //     poseData = limeb.getEntry("botpos_wpiblue").getDoubleArray(poseData);
        //     visionMeasurement = new Pose2d(poseData[0], poseData[1], getOdoYaw());
        //     swervePoseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());     

        // }


        updateOdometry();
        field.setRobotPose(swervePoseEstimator.getEstimatedPosition());

        /* 
        double loggingModulesStates [] = {
            swerveModules[0].getState().angle.getDegrees(), swerveModules[0].getState().speedMetersPerSecond * 50,
            swerveModules[1].getState().angle.getDegrees(), swerveModules[1].getState().speedMetersPerSecond * 50,
            swerveModules[2].getState().angle.getDegrees(), swerveModules[2].getState().speedMetersPerSecond * 50,
            swerveModules[3].getState().angle.getDegrees(), swerveModules[3].getState().speedMetersPerSecond * 50   
        };


        SmartDashboard.putNumber("Angle FL: ", swerveModules[0].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle FR: ", swerveModules[1].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle RL: ", swerveModules[2].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle RR: ", swerveModules[3].getState().angle.getDegrees());

        SmartDashboard.putNumber("Angle EFL: ", swerveModules[0].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle EFR: ", swerveModules[1].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle ERL: ", swerveModules[2].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle ERR: ", swerveModules[3].getCANCoderAngle().getDegrees());

            SmartDashboard.putNumberArray("SwerveModuleStates", loggingModulesStates);*/

        publisherActualStates.set(getStates());       // The states measured by encoders
        publisherTargetStates.set(getTargetStates()); // The states calculated by logic
        posePublisher.set(swervePoseEstimator.getEstimatedPosition());

    }
    
}