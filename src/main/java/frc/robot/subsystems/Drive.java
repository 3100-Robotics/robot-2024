package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Camera;
import frc.robot.Constants;
import frc.robot.Constants.driveConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
	SwerveDrive drive;

    Camera gamePieceCam;
    Camera frontTagCam;
    Camera backTagCam;


    public Drive(Camera gamePieceCam, Camera frontTagCam, Camera backTagCam) {
        this.gamePieceCam = gamePieceCam;
        this.frontTagCam = frontTagCam;
        this.backTagCam = backTagCam;

        // set the swerve telemetry's verbosity
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        // create the drivetrain from the config files
        try {
            double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(driveConstants.WHEEL_DIAMETER), 
                driveConstants.DRIVE_GEAR_RATIO,
                driveConstants.DRIVE_ENCODER_RESOLUTION);
            double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                driveConstants.STEERING_GEAR_RATIO, 
                driveConstants.STEERING_ENCODER_RESOLUTION);
            // SteeringConversionFactor = 360;
            drive = new SwerveParser(swerveJsonDir).createSwerveDrive(
                driveConstants.maxSpeed, 
                SteeringConversionFactor, 
                DriveConversionFactor);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        drive.setHeadingCorrection(false);
        drive.setCosineCompensator(false);
        drive.pushOffsetsToControllers();

        defineAutoBuilder();
//        SmartDashboard.putNumber("number", drive.getSwerveController().config.maxAngularVelocity);
    }

    @Override
    public void periodic() {
        updatePose();
        drive.updateOdometry();
//        SmartDashboard.putNumber("drive pose/x", drive.getPose().getX());
//        SmartDashboard.putNumber("drive pose/y", drive.getPose().getY());
    }

    private void updatePose() {
        //  Optional<EstimatedRobotPose> frontResult = frontTagCam.getPose(drive.getPose());
        //  if (frontResult.isPresent()) {
		//  	EstimatedRobotPose camPose = frontResult.get();
		//  	drive.addVisionMeasurement(
        //          camPose.estimatedPose.toPose2d(),
        //          camPose.timestampSeconds);
		//  }
        
        Optional<EstimatedRobotPose> backResult = backTagCam.getPose(drive.getPose());
        if (backResult.isPresent()) {
		 	EstimatedRobotPose camPose = backResult.get();
		 	drive.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds);
		 }
    }

    public Rotation2d getRoll() {
        return drive.getRoll();
    }

    public Rotation2d getPitch() {
        return drive.getPitch();
    }

    public Rotation2d getYaw() {
        return drive.getYaw();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		drive.drive(translation, rotation, fieldRelative, false);
	}

    public Command driveWithJoysticks(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotation, BooleanSupplier FieldRelative) {
        return this.run(() -> drive.drive(new Translation2d(
            -MathUtil.applyDeadband(Math.pow(vX.getAsDouble(),3), 0.04)*driveConstants.maxSpeed,
            -MathUtil.applyDeadband(Math.pow(vY.getAsDouble(), 3), 0.04)*driveConstants.maxSpeed),
            -MathUtil.applyDeadband(Math.pow(rotation.getAsDouble(), 3), 0.09) * 157,
            FieldRelative.getAsBoolean(), false));
    }

    public void lock() {
        drive.lockPose();
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPitch()));
        // drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, true, false);
    }

    /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    drive.resetOdometry(initialHolonomicPose);
  }

  public Command driveToNote(BooleanSupplier collected) {
      return this.run(() -> {
          PhotonTrackedTarget target = gamePieceCam.getBestTarget();
          double angle = 0;
          if (target != null) {
              angle = target.getYaw();
              drive(new Translation2d(driveConstants.autoCollectForwardVel, 0),
                      Math.min(angle*-driveConstants.autoCollectTurnP, driveConstants.autoCollectMaxTurnVel),
                      false);
          }
      }).until(collected);
  }

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                0.5, 4.0,
                drive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0,
                0.0);
    }

    public ChassisSpeeds getTargetSpeeds(double xSpeed, double ySpeed, double angle) {
        return drive.swerveController.getTargetSpeeds(
                xSpeed,
                ySpeed,
                angle,
                drive.getYaw().getRadians(),
                driveConstants.maxSpeed);
    }

    public double speakerShootAngle() {
      Pose2d robotPose = getPose();
      ChassisSpeeds robotVel = drive.getRobotVelocity();
      Translation2d speakerPose;
      if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
              speakerPose = new Translation2d(0, Constants.Field.blueSpeakerY);
          }
          else{
              speakerPose = new Translation2d(0, Constants.Field.redSpeakerY);
          }
      }
      else {
          speakerPose = new Translation2d(0, 0);
      }
      Rotation2d angle = speakerPose.minus(robotPose.getTranslation()).getAngle();
      Translation2d negVel = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond).times(-1);
      Translation2d shooterVel = new Translation2d(1, angle);

      return shooterVel.plus(negVel).getAngle().getDegrees();
    }

    public void defineAutoBuilder() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry,
                drive::getRobotVelocity,
                drive::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Translation PID constants
                        new PIDConstants(80,
                                0,
                                0),
                        1,
                        drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig(false, false)),
                () -> {var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();},
                this);


	}

    public Command createTrajectory(String pathName, boolean setOdomToStart) {

//        if (setOdomToStart)
//        {
//            resetOdometry(new Pose2d(path.getPoint(0).position, getYaw()));
//        }

        return new PathPlannerAuto(pathName);
	}

    public Command loadChoreoTrajectory(String pathName, Boolean setOdomToStart) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);

        if (setOdomToStart)
        {
            resetOdometry(new Pose2d(path.getPoint(0).position, getYaw()));
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
}
