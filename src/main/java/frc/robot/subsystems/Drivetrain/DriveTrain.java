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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
                    new PIDConstants(5, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(2, 0, 0)
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


    // In DriveTrain.java
    public void resetPose(Pose2d pose) {
    // 1. Reset the Odometry
        swerveOdometry.resetPosition(getOdoYaw(), getPositions(), pose);
    
    // 2. IMPORTANT: Reset the Pose Estimator too!
        swervePoseEstimator.resetPosition(getOdoYaw(), getPositions(), pose);
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
        // 1. Update the Simulated Gyro (Pigeon 2)
        // We calculate how fast the robot is rotating based on the target swerve states
        ChassisSpeeds speeds = Constants.Swerve.swerveOdoKinematics.toChassisSpeeds(getTargetStates());
        
        // 0.02 is the standard loop time (20ms)
        double dt = 0.02; 
        
        // Update Pigeon SimState (Yaw is in degrees)
        double changeInYawDegrees = Math.toDegrees(speeds.omegaRadiansPerSecond * dt);
        gyro.getSimState().addYaw(changeInYawDegrees);

        // 2. Update Each Swerve Module
        for (SwerveModule module : swerveModules) {
            module.simulationPeriodic(dt);
        }
        
        // 3. Update Odometry
        // We do NOT need to manually set the robot pose here anymore.
        // Because we updated the "Simulated Encoders" in step 2, the standard updateOdometry() 
        // running in periodic() will effectively "see" the robot moving and update the pose naturally.
    }

    @Override
    public void periodic() {    
                
        SmartDashboard.putString("Odometry: ", swerveOdometry.getPoseMeters().getTranslation().toString());
        SmartDashboard.putNumber("gyro", getYaw().getDegrees());
        SmartDashboard.putNumber("gyro Odo", getOdoYaw().getDegrees());

    
        updateOdometry();
        field.setRobotPose(swervePoseEstimator.getEstimatedPosition());

        publisherActualStates.set(getStates());       
        publisherTargetStates.set(getTargetStates()); 
        posePublisher.set(swervePoseEstimator.getEstimatedPosition());

    }
    
}