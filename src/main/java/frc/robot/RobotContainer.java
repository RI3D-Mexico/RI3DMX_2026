// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.TeleopLauncherControl;
import frc.robot.subsystems.Drivetrain.DriveTrain;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Launcher.Launcher;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);
  private final DriveTrain m_NewDriveTrain = new DriveTrain();
  private final Launcher m_Launcher = new Launcher();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();


  private Trigger launcherTrigger = new Trigger(()-> driverController.getAButtonPressed());

  public RobotContainer() {

    registerCommands();



    DriverStation.silenceJoystickConnectionWarning(true);
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Test", new PathPlannerAuto("Ri3D_Test_Auto"));
    autoChooser.addOption("SANITY CHECK", new InstantCommand(() -> System.out.println("!!! AUTO IS WORKING !!!")));


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    m_Launcher.setDefaultCommand(new TeleopLauncherControl( m_Launcher,() -> driverController.getRightTriggerAxis()));

    m_NewDriveTrain.setDefaultCommand(new DriveTrainCommand(m_NewDriveTrain, 
                                                               ()-> -driverController.getLeftY(), 
                                                               ()-> -driverController.getLeftX(),
                                                               ()-> -driverController.getRightX(),
                                                               ()-> driverController.getBButtonPressed(),
                                                               ()-> driverController.getYButtonPressed()));
    
    //configureBindings();
  }

  public void configureBindings(){

    //launcherTrigger.onTrue(new LaunchCommand(m_Launcher, ()-> driverController.getRightTriggerAxis()));
    driverController..toggleOnTrue(new IntakeCommand(m_Intake, driverController::getXButton));

  }

  public void registerCommands(){

    NamedCommands.registerCommand("Print", new PrintCommand("Initializing..."));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
