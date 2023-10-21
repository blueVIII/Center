package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RightSide2", group = "Concept")
public class RightSide2 extends LinearOpMode{
    //movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int targetPosition = 1;
    int currentPosition;
    int cycle;
    int numberLoop;
    int cones;

    //true == up
    boolean direction = true;
    double beginTime, currentTime, timeRemaining;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    int distanceParking = -15;


    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double distanceInches = 1;
        double placeHolderDistance = 1;
        // servo position
        drive.clawServo.setPosition(0.5);
        sleep(2000);

        Pose2d startingPose = new Pose2d(35,-70,90);
        drive.setPoseEstimate(startingPose);

        // all of our trajectories
        // drive forward


        TrajectorySequence beginning = drive.trajectorySequenceBuilder(startingPose)
                .forward(48.5,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addDisplacementMarkerOffset(-42, () -> {
                    targetPosition = 2500;
                })
                .strafeLeft(30)
                .build();
        drive.followTrajectorySequenceAsync(beginning);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.setMsTransmissionInterval(50);

        // april tag initialization
        while (!isStarted() && !isStopRequested()) {
            timer.reset();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0){
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT){
                        distanceParking = -15;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    } else if (tag.id == MIDDLE){
                        distanceParking = 10;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    } else if (tag.id == RIGHT) {
                        distanceParking = 35;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

        cones = 0;
        cycle = 0;
        numberLoop = 1;

        waitForStart();

        while (opModeIsActive()) {
            distanceInches = drive.distSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance Inches", distanceInches);
            telemetry.addData("current position", currentPosition);
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("Cycles", cycle);
            telemetry.addData("Parking Location", distanceParking);
            telemetry.update();
            if (distanceInches < 10 && numberLoop == 1) {
                sleep(50);
                drive.setMotorPower(0);
                placeHolderDistance = distanceInches;
                drive.breakFollowing();
                startingPose = new Pose2d(0, 0, 0);
                drive.setPoseEstimate(startingPose);
                TrajectorySequence distanceSensor = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            targetPosition = 2900 + (cycle * 100);
                            direction = true;
                        })
                        .forward(placeHolderDistance - 3.25)
                        .waitSeconds(0.025)
                        .addTemporalMarker(() -> {
                            drive.clawServo.setPosition(0.26);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.5, () -> cones++)
                        .build();
                drive.followTrajectorySequenceAsync(distanceSensor);
                numberLoop++;
            }
            if(cones == 4) {
                targetPosition = 0;
                direction = false;
                numberLoop = 0;
                drive.breakFollowing();
                startingPose = new Pose2d(0, 0, Math.toRadians(90));
                drive.setPoseEstimate(startingPose);
                TrajectorySequence parking = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            drive.liftMotor.setPower(-0.2);
                        })
                        .back(3.5,
                                SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .lineToLinearHeading(new Pose2d(distanceParking, -4, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        /*
                        .back(3)
                        .lineToLinearHeading(new Pose2d(distanceParking, 0, Math.toRadians(90)))
                        */

                        .build();
                drive.followTrajectorySequence(parking);
            }
            if (numberLoop == 2 && !drive.isBusy()) {
                startingPose = new Pose2d(24.30, -9.17, Math.toRadians(90.00));
                drive.setPoseEstimate(startingPose);
                TrajectorySequence goToCone = drive.trajectorySequenceBuilder(startingPose)
                        .back(3, SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .UNSTABLE_addDisplacementMarkerOffset(-3, () -> {
                            targetPosition = 650 - (cycle * 55);
                            direction = false;
                        })
                        .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))

                        .forward(27,
                                SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .addTemporalMarker(() -> {
                            drive.clawServo.setPosition(0.5);
                        })
                        .build();
                drive.followTrajectorySequenceAsync(goToCone);
                numberLoop++;
            }
            if (numberLoop == 3 && !drive.isBusy()) {
                startingPose = new Pose2d(62, -12, Math.toRadians(0));
                drive.setPoseEstimate(startingPose);
                TrajectorySequence backToPole = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            targetPosition = 2500;
                            direction = true;
                        })
                        .lineToLinearHeading(new Pose2d(35, -15, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .turn(Math.toRadians(90))
                        .strafeLeft(30)
                        .UNSTABLE_addDisplacementMarkerOffset(-27, () -> {
                            cycle++;
                            numberLoop = 1;
                        })
                        .build();
                drive.followTrajectorySequenceAsync(backToPole);
            }
            drive.update();
            liftUpdate(drive);
        }
    }

    public void liftUpdate(SampleMecanumDrive drive) {
        currentPosition = drive.liftMotor.getCurrentPosition();
        if (targetPosition == 0){
        }
        else if (currentPosition < targetPosition && direction == true) {
            double power = returnPower(targetPosition, drive.liftMotor.getCurrentPosition());
            drive.liftMotor.setPower(power);
        } else if (currentPosition > targetPosition && direction == false) {
            double power = returnPower(targetPosition, drive.liftMotor.getCurrentPosition());
            drive.liftMotor.setPower(power);
        }
        else if (currentPosition+10 > targetPosition && direction == true){
            drive.liftMotor.setPower(0.05);
        }
        else if (currentPosition+10 < targetPosition && direction == false){
            drive.liftMotor.setPower(0.05);
        }
    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
