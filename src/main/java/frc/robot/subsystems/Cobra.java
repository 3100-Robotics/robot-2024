package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.cobraConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class Cobra extends SubsystemBase {
    private final TalonFX pivotMotor1 = new TalonFX(cobraConstants.pivotMotor1ID);
//    private final TalonFX pivotMotor2 = new TalonFX(cobraConstants.pivotMotor2ID);
    private final TalonFX squisherMotor = new TalonFX(cobraConstants.squisherMotorID);
    private final CANSparkMax indexerMotor = new CANSparkMax(
            cobraConstants.indexerMotorID,
            MotorType.kBrushless);

    private final SparkPIDController indexerController;

    private final LaserCan laserCan1 = new LaserCan(cobraConstants.laserCan1ID);
    private final LaserCan laserCan2 = new LaserCan(cobraConstants.laserCan2ID);

    private final CANcoder pivotEncoder = new CANcoder(cobraConstants.pivotEncoderID);

    public Boolean useCurrentControl = false;

    private final LinearFilter indexerCurrent = LinearFilter.movingAverage(20);

    private double setpoint = 0;

    public Cobra() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        TalonFXConfiguration squisherConfigs = new TalonFXConfiguration();

        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cobraConstants.upperPivotSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cobraConstants.lowerPivotSoftLimit;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.pivotMotorCurrentLimit;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.Audio.BeepOnConfig = true;
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfigs.Feedback.RotorToSensorRatio = cobraConstants.pivotGearRatio;
        pivotConfigs.Feedback.SensorToMechanismRatio = 1;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = cobraConstants.pivotMotorAcceleration;
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = cobraConstants.pivotMotorVelocity;
        pivotConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        pivotConfigs.Slot0.kP = 20; // 20
        pivotConfigs.Slot0.kI = 2.8; // 2
        pivotConfigs.Slot0.kD = 0.75;
        pivotConfigs.Slot0.kG = 0.32;
        pivotConfigs.Slot0.kS = 2.6;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        squisherConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        squisherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        squisherConfigs.CurrentLimits.SupplyCurrentLimit = cobraConstants.squisherMotorCurrentLimit;
        squisherConfigs.Audio.BeepOnConfig = true;
        squisherConfigs.MotionMagic.MotionMagicAcceleration = cobraConstants.squisherMotorAcceleration;
        squisherConfigs.MotionMagic.MotionMagicCruiseVelocity = cobraConstants.squisherMotorVelocity;
        squisherConfigs.Slot0.kP = 0.5;
        squisherConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

        pivotMotor1.getConfigurator().apply(pivotConfigs);
//        pivotMotor2.getConfigurator().apply(pivotConfigs);
//        pivotMotor2.setControl(new Follower(pivotMotor1.getDeviceID(), false));

        squisherMotor.getConfigurator().apply(squisherConfigs);

        CANcoderConfiguration pivotEncoderConfigs = new CANcoderConfiguration();

        pivotEncoderConfigs.MagnetSensor.MagnetOffset = 0.339;
        pivotEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        pivotEncoder.getConfigurator().apply(pivotEncoderConfigs);

        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(cobraConstants.indexerMotorCurrentLimit);

        indexerController = indexerMotor.getPIDController();

        try {
            laserCan1.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan2.setRangingMode(LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
             DriverStation.reportError("one or both laser can devices have failed to configure", false);
             useCurrentControl = true;
        }
//        if (laserCan1.getMeasurement().distance_mm < 0.1 || laserCan2.getMeasurement().distance_mm < 0.1) {
//            useCurrentControl = true;
//            DriverStation.reportError("one or both laser can devices have failed to give a correct measurement", false);
//        }
    }

    @Override
    public void periodic() {
        indexerCurrent.calculate(indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("pivot error", pivotMotor1.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putBoolean("at pivot setpoint", atPivotPoseSetpoint());
        SmartDashboard.putNumber("pivot pos", pivotMotor1.getPosition().getValueAsDouble());



        SmartDashboard.putBoolean("laser can 1 activated", laserCan1Activated());
        SmartDashboard.putBoolean("laser can 2 activated", laserCan2Activated());

        SmartDashboard.putNumber("pivot current draw", pivotMotor1.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("squisher current draw", squisherMotor.getSupplyCurrent().getValueAsDouble());

//        SmartDashboard.putNumber("pivot angle", pivotMotor1.get)

//        SmartDashboard.putNumber("laser can 1 distance", laserCan1.getMeasurement().distance_mm);
//        SmartDashboard.putNumber("laser can 2 distance", laserCan2.getMeasurement().distance_mm);
    }

    public Boolean laserCan2Activated() {
        LaserCan.Measurement measurement = laserCan2.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm < cobraConstants.laserCanDetectionTolerance;
        }
        else {
            return false;
        }
    }

    public Boolean laserCan1Activated() {
        LaserCan.Measurement measurement = laserCan1.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm < cobraConstants.laserCanDetectionTolerance;
        }
        else {
            return false;
        }
    }

    public Boolean noteInPosition() {
        return laserCan2Activated() && !laserCan1Activated();
    }

    public BooleanSupplier isIndexerCurrentHigh() {
        return () -> indexerCurrent.calculate(indexerMotor.getOutputCurrent()) > 20;
    }

    // rotation motor basic setters

    public void setPivotSpeed(double speed) {
        pivotMotor1.set(speed*cobraConstants.pivotGearRatio);
    }

    public void setPivotPos(double pos) {
        pivotMotor1.setControl(new MotionMagicVoltage(pos));
        setpoint = pos;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void stopPivot() {
        pivotMotor1.stopMotor();
    }

    public Boolean atPivotPoseSetpoint() {
        return Math.abs(pivotMotor1.getClosedLoopError().getValueAsDouble()) < cobraConstants.pivotAngleTolerance;
    }

    // squisher motor basic setters

    public void setSquisher(double speed) {
        squisherMotor.set(speed);
    }

    public void setSquisherVel(double vel) {
        squisherMotor.setControl(new VelocityVoltage(vel));
    }

    public void stopSquisher() {
        squisherMotor.stopMotor();
    }

    public Boolean atSquisherSetpoint() {
        return squisherMotor.getClosedLoopError().getValueAsDouble() < cobraConstants.squisherSpeedTolerance;
    }

    // indexer motor basic setters

    public void setIndexer(double speed) {
        indexerMotor.set(speed);
    }

    public void setIndexerVel(double vel) {
        indexerController.setReference(vel, ControlType.kVelocity);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    public void setSquisherAndIndexer(double speed) {
        squisherMotor.set(speed);
        indexerMotor.set(speed);
    }

    // COMMANDS //

    // rotation motor commands

    public Command setPivotCommand(DoubleSupplier speed) {
        return this.run(() -> setPivotSpeed(speed.getAsDouble()));
    }

    public Command setPivotPosCommand(DoubleSupplier pos) {
        return this.runOnce(() -> setPivotPos(pos.getAsDouble())).
                andThen(Commands.waitSeconds(0.25)).
                andThen(Commands.waitUntil(this::atPivotPoseSetpoint));
    }

    public Command stopPivotCommand() {
        return this.runOnce(this::stopPivot);
    }

    // squisher motor commands

    public Command setSquisherCommand(DoubleSupplier speed) {
        return this.run(() -> setSquisher(speed.getAsDouble()));
    }

    public Command setSquisherVelCommand(DoubleSupplier vel) {
        return this.runOnce(() -> setSquisherVel(vel.getAsDouble())).
                until(() -> squisherMotor.getClosedLoopError().getValueAsDouble() < cobraConstants.squisherSpeedTolerance);
    } 

    public Command stopSquisherCommand() {
        return this.runOnce(this::stopSquisher);
    }

    // indexer motor commands

    public Command setIndexerCommand(DoubleSupplier speed) {
        return this.runOnce(() -> setIndexer(speed.getAsDouble()));
    }

    public Command setIndexerVelCommand(DoubleSupplier vel) {
        return this.run(() -> setIndexerVel(vel.getAsDouble()));
    }

    public Command stopIndexerCommand() {
        return this.runOnce(this::stopIndexer);
    }

    public Command setSquisherAndIndexerCommand(DoubleSupplier speed) {
        return this.runOnce(() -> setSquisherAndIndexer(speed.getAsDouble()));
    }

    public Command cobraCollect(Command intakeCollect) {
//        if (useCurrentControl) {
//            return setPivotPosCommand(() -> cobraConstants.pivotCollectAngle)
//                    .andThen(setSquisherAndIndexerCommand(() -> -0.3))
//                    .until(isIndexerCurrentHigh())
//                    .andThen(setSquisherAndIndexerCommand(() -> -0.3))
//                        .raceWith(Commands.waitSeconds(0.5));// wait half a second more to make sure the note is fully in the cobra
//        }
        return setPivotPosCommand(() -> cobraConstants.pivotCollectAngle).
//                andThen(Commands.waitSeconds(0.2)).
                andThen(setSquisherAndIndexerCommand(() -> -0.3)
                .alongWith(intakeCollect))
                .until(this::laserCan2Activated);
    }

    public Command shootSpeaker(Supplier<Pose2d> robotPose) {
        return Commands.sequence(
                        Commands.runOnce(() -> setSquisherVel(cobraConstants.squisherShootSpeed)),
                        setPivotPosCommand(() -> { // get the angle for the pivot
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
                            double distanceFromSpeaker = speakerPose.getDistance(robotPose.get().getTranslation());
                            double height = Constants.Field.speakerZ - 0.48;

                            return Math.tan(height/distanceFromSpeaker);
                        }));
    }

    public Command scoreAmp() {
        return setPivotPosCommand(() -> cobraConstants.pivotAmpPos)
                .andThen(setSquisherCommand(() -> 0.5))
                .alongWith(setIndexerCommand(() -> 0.5));
    }
}
