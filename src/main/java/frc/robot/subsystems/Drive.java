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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    String autoRotationType = "normal";

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
        SmartDashboard.putNumber("robot needed angle", speakerShootAngle());
    }

    private void updatePose() {
          Optional<EstimatedRobotPose> frontResult = frontTagCam.getPose(drive.getPose());
          if (frontResult.isPresent()) {
		  	EstimatedRobotPose camPose = frontResult.get();
		  	drive.addVisionMeasurement(
                  camPose.estimatedPose.toPose2d(),
                  camPose.timestampSeconds);
		  }
        
//        Optional<EstimatedRobotPose> backResult = backTagCam.getPose(drive.getPose());
//        if (backResult.isPresent()) {
//		 	EstimatedRobotPose camPose = backResult.get();
//		 	drive.addVisionMeasurement(
//                camPose.estimatedPose.toPose2d(),
//                camPose.timestampSeconds);
//		 }
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
          else {
              drive(new Translation2d(0, 0), 0, false);
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
//      Pose2d robotPose = getPose();
//      ChassisSpeeds robotVel = drive.getRobotVelocity();
//      Translation2d speakerPose;
//      if (DriverStation.getAlliance().isPresent()) {
//          if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
//              speakerPose = new Translation2d(0, Constants.Field.blueSpeakerY);
//          }
//          else{
//              speakerPose = new Translation2d(0, Constants.Field.redSpeakerY);
//          }
//      }
//      else {
//          speakerPose = new Translation2d(0, Constants.Field.blueSpeakerY);
//      }
//
//      Translation2d speakerRelativePose = robotPose.getTranslation().minus(speakerPose);
//      double angle = Math.tan(speakerRelativePose.getX()/speakerRelativePose.getY());
////      Translation2d negVel = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond).times(-1);
////      Translation2d shooterVel = new Translation2d(1, angle);
//
////      return shooterVel.plus(negVel).getAngle().getRadians();
//        return angle;



        Pose2d dtvalues = this.getPose();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        double shootingSpeed = Constants.cobraConstants.squisherShootSpeed;
        double deltaX, deltaY, speakerDist, shootingTime, currentXSpeed, currentYSpeed;
        Translation2d targetOffset;

        //triangle for robot angle
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            deltaY = Math.abs(dtvalues.getY() - Constants.Field.redSpeakerY);
        } else {
            deltaY = Math.abs(dtvalues.getY() - Constants.Field.redSpeakerY);
        }
        deltaX = Math.abs(dtvalues.getY() - Constants.Field.speakerX);
        speakerDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        // SmartDashboard.putNumber("dist to speakre", speakerDist);

        Rotation2d unadjustedAngle = Rotation2d.fromRadians(Math.asin(deltaX/speakerDist));
        double totalDistToSpeaker = Math.sqrt(Math.pow(Constants.Field.speakerZ-0.48, 2) + Math.pow(speakerDist, 2));
        shootingTime = totalDistToSpeaker/shootingSpeed; //calculates how long the note will take to reach the target
        currentXSpeed = drive.getRobotVelocity().vxMetersPerSecond;
        currentYSpeed = drive.getRobotVelocity().vyMetersPerSecond;
        targetOffset = new Translation2d(currentXSpeed*shootingTime*5*unadjustedAngle.getRadians(), currentYSpeed*shootingTime*5);
        //line above calculates how much our current speed will affect the ending location of the note if it's in the air for ShootingTime

        //next 3 lines set where we actually want to aim, given the offset our shooting will have based on our speed
        int correctionDirection;
        double speakerY;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            correctionDirection = 1;
            speakerY = Constants.Field.blueSpeakerY;
        } else {
            correctionDirection = -1;
            speakerY = Constants.Field.blueSpeakerY;
        }
        double offsetSpeakerY = speakerY+(targetOffset.getY()*correctionDirection);
        double offsetSpeakerX = Constants.Field.speakerX+(targetOffset.getX()*correctionDirection);
        double offsetDeltaX = Math.abs(dtvalues.getX() - offsetSpeakerX);
        double offsetDeltaY = Math.abs(dtvalues.getY() - offsetSpeakerY);

        Translation2d adjustedTarget = new Translation2d(offsetSpeakerX, offsetSpeakerY);
        double offsetSpeakerdist = Math.sqrt(Math.pow(offsetDeltaY, 2) + Math.pow(offsetDeltaX, 2));
        SmartDashboard.putNumber("offsetSpeakerDis", offsetSpeakerdist);
        // SmartDashboard.putString("offset amount", targetOffset.toString());
        // SmartDashboard.putString("offset speaker location", new Translation2d(offsetSpeakerX, offsetSpeakerY).toString());
        double m_desiredRobotAngle;
        //getting desired robot angle
        if (alliance.get() == Alliance.Blue) {
            if (dtvalues.getY() >= adjustedTarget.getY()) {
                double thetaAbove = -Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))-90;
                m_desiredRobotAngle = thetaAbove;
            }
            else{
                double thetaBelow = Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))+90;
                m_desiredRobotAngle = thetaBelow;
            } } else {
            if (dtvalues.getY() >= adjustedTarget.getY()) {
                double thetaAbove = Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))-90;
                m_desiredRobotAngle = thetaAbove;
            }
            else{
                double thetaBelow = -Math.toDegrees(Math.asin(offsetDeltaX / offsetSpeakerdist))+90;
                m_desiredRobotAngle = thetaBelow;
            }
        }
        SmartDashboard.putString("adjusted target", adjustedTarget.toString());
        return m_desiredRobotAngle;
    }

    public Command rotateToSpeaker() {
      return this.run(() -> drive(new Translation2d(0, 0),
              Math.min(speakerShootAngle()*50, driveConstants.autoCollectMaxTurnVel),
              false));
    }

    public Command setAutoRotationOverrideType(String type) {
      return Commands.runOnce(() -> autoRotationType = type);
    }

    private Optional<Rotation2d> autoRotationOverride() {
      if (autoRotationType.equals("speaker")) {
          return Optional.of(new Rotation2d(speakerShootAngle()));
      }
      else if (autoRotationType.equals("object")) {
          PhotonTrackedTarget target = gamePieceCam.getBestTarget();
          double angle = 0;
          if (target != null) {
              angle = Math.min(target.getYaw()*-driveConstants.autoCollectTurnP, driveConstants.autoCollectMaxTurnVel);
              return Optional.of(new Rotation2d(Units.degreesToRadians(angle)));
          }
          else {
              return Optional.empty();
          }
      }


      return Optional.empty();
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
                        7,
                        drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig(true, true)),
                () -> {var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();},
                this);

	    com.pathplanner.lib.controllers.PPHolonomicDriveController.setRotationTargetOverride(this::autoRotationOverride);
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

        Command auto = AutoBuilder.followPath(path);

        if (setOdomToStart)
        {
//            resetOdometry(new Pose2d(path.getPoint(0).position, getYaw()));
            return Commands.runOnce(() -> resetOdometry(new Pose2d(path.getPoint(0).position, getYaw()))).andThen(auto);
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
}
