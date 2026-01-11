// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.Drivetrain.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;


public class RobotContainer {

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);
  private final DriveTrain m_NewDriveTrain = new DriveTrain();

  public RobotContainer() {
    
    m_NewDriveTrain.setDefaultCommand(new DriveTrainCommand(m_NewDriveTrain, 
                                                               ()-> -driverController.getLeftY(), 
                                                               ()-> -driverController.getLeftX(),
                                                               ()-> -driverController.getRightX(),
                                                               ()-> driverController.getBButtonPressed(),
                                                               ()-> driverController.getYButtonPressed()));
  }
}
