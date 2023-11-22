package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.drive.drives.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@Autonomous(name = "SpikeTest")
public class SpikeTest extends LinearOpMode {
    public int spike = 2;
    public static double SPIKE_POINT_X = 12;
    public static double SPIKE_POINT_Y = -34.5;
    public static double HEADING = 90;
    double BACKDROPY = -35;
    public static double POINTX = 45;
    public static double POINTY = -60;
    public static double POINTHEADING = 90;
    public static boolean audience = false;
    double BACKDROPX = 50.75;
    int backdropCenterAT = 5;
    int audienceAT = 8;

    public ArrayList<Endpoint> endpoints = new ArrayList<Endpoint>();

    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
    public Pose2dWrapper resetPose = new Pose2dWrapper(13, -51.5, -90);
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -50.5, 45);
    public Pose2dWrapper audiencePose = new Pose2dWrapper(-58, -41.5, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(36, -37.5, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-58, -8.5, 0);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
    //X values get wonky here, as invertLeft is ran on all Endpoints used on the left starting point,
    //so numbers are less than would be expected and sometimes greater than 70, however invertLeft() clears this up,
    //decided to do this for readability in the if(LEFT) statement
    public Pose2dWrapper travelPose = new Pose2dWrapper(35, -8.5, 0);
    public Pose2dWrapper aprilTagPose = new Pose2dWrapper(50, -37, 0);
    public Pose2dWrapper pixelPose = new Pose2dWrapper(-58, -36.5, 0);
    public Pose2dWrapper postPixelPose = new Pose2dWrapper(-59.75, -36.5, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(-59 , -36.5, 0);


    ComputerVision vision;
    ArrayList<AprilTagDetection> aprilTagDetections;
    HashMap<Integer, AprilTagPoseFtc> aprilTagTranslations = new HashMap<>();
    InputHandler inputHandler;
    Servo armServo;

    boolean inputComplete = false;
    boolean isBlue = false;
    Pose2d robotPose;

    Servo leftFlipper;
    Servo rightFlipper;
    DcMotorSimple intake;
    ElapsedTime runtime = new ElapsedTime();
    int i = 1; //used as an iterator for outputLog()



    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        inputHandler = InputAutoMapper.normal.autoMap(this);
        while(inputComplete == false){
            inputHandler.loop();
            if(inputHandler.up("D1:DPAD_LEFT")){
                isBlue = !isBlue;
            }
            if(inputHandler.up("D1:DPAD_RIGHT")){
                audience = !audience;
            }
            if(inputHandler.up("D1:X")){
                inputComplete = true;
            }
            telemetry.addData("is Blue:", isBlue);
            telemetry.addData("is Near Audience:", audience);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }
        vision = new ComputerVision(hardwareMap);
        armServo = hardwareMap.get(Servo.class, "Arm");
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        leftFlipper = hardwareMap.get(Servo.class, "leftHook");
        rightFlipper = hardwareMap.get(Servo.class, "rightHook");

        leftFlipper.setPosition(0.999);
        rightFlipper.setPosition(0.01);

        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0);

        while(opModeInInit()) {
            vision.updateTensorFlow();
            spike = vision.checkSpike(isBlue, audience);
            sleep(1);
            telemetry.addData("spike: ", spike);
            telemetry.update();
        }


        if (isStopRequested()) return;
        switch (spike) {
            case 1:
                SPIKE_POINT_X = 3.7;
                SPIKE_POINT_Y = -34.5;
                HEADING = 115;
                aprilTagPose.y = -28.3;
                break;
            case 2:
                SPIKE_POINT_X = 11;
                SPIKE_POINT_Y = -36.5;
                HEADING = 90;
                aprilTagPose.y = -35.5;
                break;
            case 3:
                SPIKE_POINT_X = 17.5;
                SPIKE_POINT_Y = -34.5;
                HEADING = 65;
                aprilTagPose.y = -42.4;
                break;
        }


        Pose2dWrapper spikePose = new Pose2dWrapper(SPIKE_POINT_X, SPIKE_POINT_Y, HEADING);


        if(audience){
            startPose.x = -39;
            tempParkPose.y = -13.5;
            BACKDROPX = 47.75;

            spikePose.x = -(spikePose.x + 34);
            mediaryPose.x = -(mediaryPose.x + 34);
        }
        if(isBlue){
            startPose.y *= -1;
            startPose.heading *= -1;
            audienceAT = 10;
            backdropCenterAT = 2;

            spikePose.y *= -1;
            spikePose.heading *= -1;
            mediaryPose.y *= -1;
            mediaryPose.heading *= -1;
            backdropPose.y *= -1;
            backdropPose.heading *= -1;
            centerPose.y *= -1;
            centerPose.heading *= -1;
            avoidancePose.y *= -1;
            avoidancePose.heading *= -1;
            travelPose  .y *= -1;
            travelPose.heading *= -1;
            audiencePose.y *= -1;
            audiencePose.heading *= -1;
            tempParkPose.y *= -1;
            tempParkPose.heading *= -1;
            aprilTagPose.y *= -1;
            aprilTagPose.heading *= -1;
            pixelPose.y *= -1;
            pixelPose.heading *= -1;
            postPixelPose.y *= -1;
            postPixelPose.heading *= -1;
        }

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                .build();
        Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end())
                .strafeTo(mediaryPose.toPose2d().vec())
                .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                .build();

        Trajectory audienceTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .lineToLinearHeading(audiencePose.toPose2d())
                .build();

        /*
        Trajectory pixelTraj = drive.trajectoryBuilder(audienceTraj.end())
                .strafeTo(pixelPoint.getPos())
                .build();
         */

        Trajectory pixelTraj = drive.trajectoryBuilder(audienceTraj.end())
                .splineTo(pixelPose.toPose2d().vec(), Math.toRadians(pixelPose.heading))
                .build();
        //Compound Trajectories for testing
        Trajectory postSpikeTraj = drive.trajectoryBuilder(spikeTraj.end())
                .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(mediaryPose.heading))
                .splineTo(audiencePose.toPose2d().vec(), Math.toRadians(audiencePose.heading))
                .splineTo(pixelPose.toPose2d().vec(), Math.toRadians(pixelPose.heading))
                        .build();

        outputLog(drive); //1
        drive.followTrajectory(spikeTraj);
        if(!audience) {
            outputLog(drive); //2
            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            vision.tensorFlowProcessor.shutdown();
            drive.followTrajectory(backdropTraj);
            outputLog(drive);
            Trajectory tempParkTraj = depositPixel(drive);
            drive.followTrajectory(tempParkTraj);
            outputLog(drive);
        }


        if(audience){
            outputLog(drive); //2
            drive.followTrajectory(postSpikeTraj);

            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            vision.tensorFlowProcessor.shutdown();
            drive.followTrajectory(audienceTraj);
            outputLog(drive); //4
            drive.followTrajectory(pixelTraj);
            outputLog(drive); //5

            vision.tensorFlowProcessor.shutdown();
            vision.setActiveCameraTwo();
            while(aprilTagTranslations.get(audienceAT) == null){
                vision.updateAprilTags();
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(audienceAT, false);
            }
            drive.setPoseEstimate(robotPose);
            outputLog(drive); //6
            Trajectory postPixelTraj =  drive.trajectoryBuilder(robotPose)
                    .lineToLinearHeading(postPixelPose.toPose2d(),
                            NewMecanumDrive.getVelocityConstraint(10, 1.85, 13.5),
                            NewMecanumDrive.getAccelerationConstraint(10))
                                    .build();
            Trajectory longDriveTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .strafeTo(avoidancePose.toPose2d().vec())
                    .splineToConstantHeading(centerPose.toPose2d().vec(), Math.toRadians(centerPose.heading))
                    .splineToConstantHeading(travelPose.toPose2d().vec(), Math.toRadians(travelPose.heading))
                    .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                            .build();

            drive.followTrajectory(postPixelTraj);
            outputLog(drive);//7

            intake.setPower(1);
            leftFlipper.setPosition(0.4);
            rightFlipper.setPosition(0.6);
            sleep(650);
            leftFlipper.setPosition(0.999);
            rightFlipper.setPosition(0.01);

            /*
            Trajectory centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .strafeTo(centerPoint.getPos())
                    .build();
            Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                    .lineToLinearHeading(travelPoint.getPose())
                    .build();
            Trajectory leftBackdropTraj = drive.trajectoryBuilder(travelTraj.end())
                    .strafeTo(backdropPoint.getPos())
                    .build();


            drive.followTrajectory(centerTraj);
            outputLog(drive);//8
            drive.followTrajectory(travelTraj);
            outputLog(drive);//9
            drive.followTrajectory(leftBackdropTraj);
            outputLog(drive);
            */

            drive.followTrajectory(longDriveTraj);
            Trajectory tempParkTraj = depositPixel(drive);
            drive.followTrajectory(tempParkTraj);
            outputLog(drive);
        }
    }
    public Trajectory depositPixel(NewMecanumDrive drive){
        intake.setPower(0);
        vision.setActiveCameraOne();
        while(aprilTagTranslations.get(backdropCenterAT) == null){
            vision.updateAprilTags();
            aprilTagTranslations = vision.getTranslationToTags();
            if(isBlue) {robotPose = vision.localize(2, true); } else {robotPose = vision.localize(5, true); }
        }
        drive.setPoseEstimate(robotPose);
        outputLog(drive);
        Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                .strafeTo(aprilTagPose.toPose2d().vec(),
                        NewMecanumDrive.getVelocityConstraint(30, 2, 13.5),
                        NewMecanumDrive.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(aprilTagTraj);
        outputLog(drive);
        armServo.setPosition(0.275);
        sleep(2250);
        armServo.setPosition(0.04);
        Trajectory tempParkTraj = drive.trajectoryBuilder(aprilTagTraj.end())
                .strafeTo(tempParkPose.toPose2d().vec())
                .build();
        return tempParkTraj;
    }
    public void outputLog(NewMecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()), runtime.milliseconds(), i);
        i++;
    }
}
