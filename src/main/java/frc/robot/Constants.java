// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
    }

    public static class driveConstants {

        public static final double ROBOT_MASS = 45.35924; // 32lbs * kg per pound

        // a matter var for limiting velocity
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0,
                Units.inchesToMeters(4)), ROBOT_MASS);

        public static final Transform3d frontTagCamPose = new Transform3d(
                Units.inchesToMeters(12.25),
                0,
                Units.inchesToMeters(8.25),
                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));
        public static final Transform3d backTagCamPose = new Transform3d(
                Units.inchesToMeters(-7.5),
                Units.inchesToMeters(-11),
                Units.inchesToMeters(12.5),
                new Rotation3d(0, Units.degreesToRadians(45), 0));

        // loop time to use
        public static final double maxSpeed = Units.feetToMeters(14.5);

        public static final double WHEEL_DIAMETER = 3;
        public static final double DRIVE_GEAR_RATIO = 4.714285714285714;
        public static final double DRIVE_ENCODER_RESOLUTION = 1.0;

        public static final double STEERING_GEAR_RATIO = 1;
        public static final double STEERING_ENCODER_RESOLUTION = 1;

        public static final double autoCollectForwardVel = 0.65;
        public static final double autoCollectTurnP = 4;
        public static final double autoCollectMaxTurnVel = 200;
    }

    public static class collectorConstants {
        public static final int collectorMotorID = 9;

        public static final int collectorMotorCurrentLimit = 40;
        public static final double collectorMotorP = 0.5;
    }

    public static class climberConstants {
        public static final int climberMotor1ID = 10;
        public static final int climberMotor2ID = 11;

        public static final double lowerSoftLimit = 0;
        public static final double upperSoftLimit = 100;

        public static final double climberMotorCurrentLimit = 40;

        public static final double climberMotorAcceleration = 2;
        public static final double climberMotorVelocity = 0.5;
    }

    public static class cobraConstants {
        public static final int pivotMotor1ID = 12;
        public static final int pivotMotor2ID = 13;
        public static final int squisherMotorID = 14;
        public static final int indexerMotorID = 15;

        public static final int laserCan1ID = 17;
        public static final int laserCan2ID = 16;
        public static final int pivotEncoderID = 18;

        public static final double pivotMotorCurrentLimit = 40;
        public static final double squisherMotorCurrentLimit = 40;
        public static final int indexerMotorCurrentLimit = 20;

        public static final double lowerPivotSoftLimit = 0;
        public static final double upperPivotSoftLimit = 1.003;
        public static final double pivotMotorAcceleration = 0.408;
        public static final double pivotMotorVelocity = 50;

        public static final double pivotGearRatio = 61.5385;
        public static final double indexerGearRatio = 12 * 1.125;

        public static final double pivotCollectAngle = 0.93;
        public static final double pivotFrontSubwooferAngle = 0.76;
        public static final double pivotBackSubwooferAngle = 0.92;
        public static final double pivotAmpAngle = 0.97;

        public static final double squisherMotorAcceleration = 2;
        public static final double squisherMotorVelocity = 0.5;
        public static final double pivotAngleTolerance = 0.01;

        public static final double squisherSpeedTolerance = 0.5;

        public static final double laserCanDetectionTolerance = 100;
        public static final double squisherShootSpeed = 4000;
        public static final double pivotAmpPos = 90;
    }

    public final class Field {
        public static final double blueSpeakerY = 5.612;
        public static final double redSpeakerY = 2.636;
        public static final double speakerWidth = 105;
        public static final double speakerX = 0.23;//16.412;
        public static final double speakerZ = 2.045; //height of opening
        public static final double blueAmpX = 0;
        public static final double redAmpX = 0;
        public static final double AmpY = 0;
    }

    public final class LEDConstants {
        public static final int port = 0;
        public static final int length = 69;
    }

    public enum LEDStates {
        collecting() {
            @Override
            public int getR() {
                return 200;
            }
            @Override
            public int getG() {
                return 120;
            }

            @Override
            public int getB() {
                return 50;
            }

            @Override
            public Boolean isAnimated() {
                return true;
            }

            @Override
            public String getAnimationType() {
                return "flash";
            }

            @Override
            public int getAnimationTime() {
                return 1;
            }
        },

        speaker() {
            @Override
            public int getR() {
                return 0;
            }
            @Override
            public int getG() {
                return 0;
            }

            @Override
            public int getB() {
                return 100;
            }

            @Override
            public Boolean isAnimated() {
                return false;
            }
            @Override
            public String getAnimationType() {
                return "none";
            }
        },

        amp() {
            @Override
            public int getR() {
                return 100;
            }
            @Override
            public int getG() {
                return 100;
            }

            @Override
            public int getB() {
                return 100;
            }

            @Override
            public Boolean isAnimated() {
                return false;
            }

            @Override
            public String getAnimationType() {
                return "none";
            }
        },

        nothing() {
            @Override
            public int getR() {
                return 0;
            }
            @Override
            public int getG() {
                return 100;
            }

            @Override
            public int getB() {
                return 0;
            }

            @Override
            public Boolean isAnimated() {
                return false;
            }

            @Override
            public String getAnimationType() {
                return "none";
            }
        },

        staticColor() {
            @Override
            public int getR() {
                return 0;
            }
            @Override
            public int getG() {
                return 0;
            }

            @Override
            public int getB() {
                return 255;
            }

            @Override
            public Boolean isAnimated() {
                return false;
            }

            @Override
            public String getAnimationType() {
                return "none";
            }

            @Override
            public int getAnimationTime() {
                return 0;
            }
        },

        off() {},

        rainbow() {
            @Override
            public int getR() {
                return 0;
            }
            @Override
            public int getG() {
                return 0;
            }

            @Override
            public int getB() {
                return 0;
            }

            @Override
            public Boolean isAnimated() {
                return true;
            }

            @Override
            public String getAnimationType() {
                return "rainbow";
            }
        };

        public int getR() {
            return 0;
        }

        public int getG() {
            return 0;
        }

        public int getB() {
            return 0;
        }

        public Boolean isAnimated() {
            return true;
        }

        public String getAnimationType() {
            return "none";
        }

        public int getAnimationTime() {
            return 0;
        }
    }
}
