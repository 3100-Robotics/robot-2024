// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Camera GamePieceCam = new Camera("objectDetector");
  public final Camera frontTagCam = new Camera("front_arducam_OV9281", Constants.driveConstants.frontTagCamPose);
  public final Camera backTagCam = new Camera("back_arducam_OV9281", Constants.driveConstants.backTagCamPose);
  public final Drive drive = new Drive(GamePieceCam, frontTagCam, backTagCam);
  public final Cobra cobra = new Cobra();
  public final Collector collector = new Collector();
  public final Climber climber = new Climber();
  public final LEDs leds = new LEDs();

  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private Boolean shootingInSpeaker = false;

  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  public final CommandXboxController coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(drive.driveWithJoysticks(
        driverController::getLeftY,
        driverController::getLeftX,
        driverController::getRightX,
        () -> SmartDashboard.getBoolean("is field oriented", false)));

    climber.setDefaultCommand(climber.runClimberWithJoysticks(coDriverController::getLeftY));

//    cobra.setDefaultCommand(cobra.setSquisherAndIndexerCommand(() -> 0));
    collector.setDefaultCommand(collector.setCommand(() -> 0.1));
        
    // Configure the trigger bindings
    configureBindings();
    configureAutonomous();

    SignalLogger.setPath("/media/sda1/");
  }

  private void configureAutonomous() {
    Command speakerCommand  = Commands.sequence(
            leds.setState(Constants.LEDStates.speaker),
            cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherShootSpeed),
            cobra.setPivotPosCommand(() -> 0.77));
//            cobra.setIndexerCommand(() -> 0.5),
//            Commands.waitSeconds(0.5),
//            cobra.setSquisherAndIndexerCommand(() -> 0),
//            leds.setState(Constants.LEDStates.nothing));

    Command prepShootCommand = Commands.sequence(
            cobra.setIndexerCommand(() -> 0.5),
            Commands.waitSeconds(0.5),
            cobra.setSquisherAndIndexerCommand(() -> 0),
            leds.setState(Constants.LEDStates.nothing));

//    Command speakerCommand = Commands.sequence(
//            drive.setShootingStatus(true),
//            leds.setState(Constants.LEDStates.speaker),
//            cobra.setSquisherCommand(() -> Constants.cobraConstants.squisherShootSpeed),
//            cobra.setPivotPosCommand(drive::speakerShootAngle),
//            cobra.setIndexerCommand(() -> 0.5),
//            Commands.waitSeconds(0.75),
//            cobra.setSquisherAndIndexerCommand(() -> 0),
//            cobra.setPivotPosCommand(() -> Constants.cobraConstants.pivotCollectAngle),
//            leds.setState(Constants.LEDStates.nothing),
//            drive.setShootingStatus(false));

    NamedCommands.registerCommand("shoot", speakerCommand);

    NamedCommands.registerCommand("shoot prep", prepShootCommand);

    NamedCommands.registerCommand("collect",
            cobra.cobraCollect(collector.collect(cobra::laserCan2Activated)));

    Command twoPiece = drive.loadChoreoTrajectory("2 piece", true);
    autoChooser.addOption("2 piece", twoPiece);

    Command driveAuto = drive.loadChoreoTrajectory("drive", true);
    autoChooser.addOption("drive", driveAuto);

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // collect
    driverController.leftBumper().whileTrue(
            leds.setState(Constants.LEDStates.collecting).andThen(
            Commands.parallel(
                            cobra.cobraCollect(collector.collect(cobra::laserCan2Activated)),
                            drive.driveToNote(cobra::laserCan2Activated)).
                    andThen(cobra.setSquisherAndIndexerCommand(() -> 0))).
                    andThen(leds.setState(Constants.LEDStates.nothing)));

    driverController.x().onTrue(cobra.cobraCollect(
            leds.setState(Constants.LEDStates.collecting).
            andThen(collector.collect(cobra::laserCan2Activated))).
            andThen(cobra.setSquisherAndIndexerCommand(() -> 0)).
            andThen(leds.setState(Constants.LEDStates.nothing)));

    // amp
    Command ampCommands = Commands.sequence(
            leds.setState(Constants.LEDStates.amp),
            cobra.setPivotPosCommand(() -> 0.98),
            cobra.setSquisherVelCommand(() -> 10));

    driverController.y().onTrue(ampCommands);

//    driverController.b().whileTrue(drive.driveToPose(new Pose2d(
//            new Translation2d(Constants.Field.ampX, Constants.Field.blueAmpY-1),
////            new Translation2d(Constants.Field.ampX,
////            DriverStation.getAlliance().isPresent() ?
////                    (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ?
////                            Constants.Field.blueAmpY :
////                            Constants.Field.redAmpY) : Constants.Field.blueAmpY),
//            new Rotation2d(0))).andThen(ampCommands));

    // speaker
    driverController.a().onTrue(Commands.sequence(
            leds.setState(Constants.LEDStates.speaker),
            cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherShootSpeed),
            cobra.setPivotPosCommand(() -> 0.77)));

    // execute speaker or amp
    driverController.rightBumper().onTrue(Commands.sequence(
            cobra.setIndexerCommand(() -> 0.5),
            Commands.waitSeconds(0.75),
            cobra.setSquisherAndIndexerCommand(() -> 0),
            cobra.setPivotPosCommand(() -> Constants.cobraConstants.pivotCollectAngle),
            leds.setState(Constants.LEDStates.nothing)));

//    driverController.b().onTrue(Commands.sequence(
//            Commands.runOnce(() -> leds.setStateNonCommand(Constants.LEDStates.speaker)),
//            cobra.setSquisherVelCommand(() -> Constants.cobraConstants.squisherShootSpeed),
//            cobra.setPivotPosCommand(() -> 1.226)));

    coDriverController.a().whileTrue(collector.setCommand(() -> 0.5));


    driverController.povLeft().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("is field oriented", true)));
    driverController.povRight().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("is field oriented", false)));

//    coDriverController.rightBumper().whileTrue(cobra.setIndexerCommand(() -> -0.3));
    coDriverController.rightBumper().whileTrue(Commands.sequence(
            Commands.runOnce(SignalLogger::start),
            Commands.waitSeconds(0.2),
            cobra.getSysIDCommand(),
            Commands.waitSeconds(0.2),
            Commands.runOnce(SignalLogger::stop)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
