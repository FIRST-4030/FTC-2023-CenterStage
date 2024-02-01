package org.firstinspires.ftc.teamcode.robot2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive2;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Endpoint;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.HashMap;

@Disabled
@Config
@Autonomous(name = "SpikeTest2")
public class SpikeTest2 extends LinearOpMode {
    public static int SPIKE = 0;
    public int spike = 2;
    public int audiencetag = 5;

    public static double endX;
    public static double endY;
    public static double endHeading;
    public static double SPIKE_POINT_X = 12;
    public static double SPIKE_POINT_Y = -33;
    public static double HEADING = 90;
    double BACKDROPY = -35;
    public static double POINTX = 45;
    public static double POINTY = -60;
    public static double POINTHEADING = 90;
    public static boolean audience = false;
    double BACKDROPX = 50.75;

    public ArrayList<Endpoint> endpoints = new ArrayList<Endpoint>();

    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -49, 0);
    public Pose2dWrapper audiencePose = new Pose2dWrapper(-58, -40, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(33, -36, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-58, -7, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(30.5 , -12, 90);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
    //X values get wonky here, as invertLeft is ran on all Endpoints used on the left starting point,
    //so numbers are less than would be expected and sometimes greater than 70, however invertLeft() clears this up,
    //decided to do this for readability in the if(LEFT) statement
    public Pose2dWrapper travelPose = new Pose2dWrapper(35, -7, 0);
    public Pose2dWrapper aprilTagPose = new Pose2dWrapper(50, -35.5, 0);


    ComputerVision2 vision;
    ArrayList<AprilTagDetection> aprilTagDetections;
    HashMap<Integer, AprilTagPoseFtc> aprilTagTranslations = new HashMap<>();
    InputHandler inputHandler;
    //Servo armServo;

    ElapsedTime operationTimer;
    boolean inputComplete = false;
    boolean isBlue = false;
    Pose2d robotPose;




    @Override
    public void runOpMode() throws InterruptedException {
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
        NewMecanumDrive2 drive = new NewMecanumDrive2(hardwareMap);
        vision = new ComputerVision2(hardwareMap);
        //armServo = hardwareMap.get(Servo.class, "Arm");
        operationTimer = new ElapsedTime();

        while(opModeInInit()) {
            vision.update();
            spike = vision.checkSpike(isBlue);
            sleep(1);
            telemetry.addData("spike: ", spike);
            telemetry.update();
        }


        if (isStopRequested()) return;
        switch (spike) {
            case 1:
                SPIKE_POINT_X = 8.5;
                SPIKE_POINT_Y = -37;
                HEADING = 145;
                aprilTagPose.y = -29.3;
                break;
            case 2:
                SPIKE_POINT_X = 11;
                SPIKE_POINT_Y = -35;
                HEADING = 90;
                aprilTagPose.y = -35.5;
                break;
            case 3:
                SPIKE_POINT_X = 17.5;
                SPIKE_POINT_Y = -36;
                HEADING = 55;
                aprilTagPose.y = -41.4;
                break;
        }



        Endpoint spikePoint = new Endpoint(SPIKE_POINT_X, SPIKE_POINT_Y, HEADING);
        Endpoint mediaryPoint = new Endpoint(mediaryPose.x, mediaryPose.y, mediaryPose.heading);
        Endpoint backdropPoint = new Endpoint(backdropPose.x, backdropPose.y, backdropPose.heading);
        Endpoint centerPoint = new Endpoint(centerPose.x, centerPose.y, centerPose.heading);
        Endpoint avoidancePoint = new Endpoint(avoidancePose.x, avoidancePose.y, avoidancePose.heading);
        Endpoint travelPoint = new Endpoint(travelPose.x, travelPose.y, travelPose.heading);
        Endpoint audiencePoint = new Endpoint(audiencePose.x, audiencePose.y, audiencePose.heading);
        Endpoint tempParkPoint = new Endpoint(tempParkPose.x, tempParkPose.y, tempParkPose.heading);
        Endpoint aprilPoint = new Endpoint(aprilTagPose.x, aprilTagPose.y, aprilTagPose.heading);



        if(audience){
            startPose.x += 24;
            startPose.x *= -1;
            BACKDROPX = 47.75;
            tempParkPose.y = -13.5;




            //Change heading value to match mirrored spikes
            /*if (HEADING == 145){
                spikePoint.setHeading(55);
            } else if (HEADING == 55){
                spikePoint.setHeading(145);
            }
             */

            spikePoint.invertSpikeLeft();
            mediaryPoint.invertLeft();
        }
        if(isBlue){
            startPose.y *= -1;
            startPose.heading *= -1;
            audiencetag = 2;


            spikePoint.invertSides();
            mediaryPoint.invertSides();
            backdropPoint.invertSides();
            centerPoint.invertSides();
            avoidancePoint.invertSides();
            travelPoint.invertSides();
            audiencePoint.invertSides();
            tempParkPoint.invertSides();
            aprilPoint.invertSides();
        }



        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(spikePoint.getPos(), Math.toRadians(spikePoint.getHeading()))
                .build();
        /*Trajectory resetTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                        .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                                .build();
         */
        Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end())
                .strafeTo(mediaryPoint.getPos())
                .build();
        Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .splineTo(backdropPoint.getPos(), Math.toRadians(backdropPoint.getHeading()))
                .build();

        //modify to avoid wing and end with a heading of 0 and target further towards center
        /*Trajectory combinedTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                .splineTo(resetPoint.getPos(), Math.toRadians(resetPoint.getHeading()))
                .splineTo(mediaryPoint.getPos(), Math.toRadians(mediaryPoint.getHeading()))
                .build();
         */

        //modify to strafe w/ heading of 0
        //reverse to pixels
        //intake pixel (turn intake on, use flippers)
        //disable intake

        Trajectory audienceTraj = drive.trajectoryBuilder(mediaryTraj.end())
                .lineToLinearHeading(audiencePoint.getPose())
                .build();
        Trajectory centerTraj = drive.trajectoryBuilder(audienceTraj.end())
                .strafeTo(centerPoint.getPos())
                .build();
        Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                .lineToLinearHeading(travelPoint.getPose())
                .build();
        Trajectory leftBackdropTraj = drive.trajectoryBuilder(travelTraj.end())
                .strafeTo(backdropPoint.getPos())
                .build();


        /*
        after BackdropTraj:
        -select AprilTag to focus on and move to that position
        -move arm to deposit pixels
        -lower arm

        Loop the following:
        reverse back to x-coord and heading of travelTraj target
        reverse to pixel stack
        intake one pixel, then intake another pixel,
        repeat the already programmed auto steps of travelTraj and backdropTraj
        deposit pixel

        Long term idea:
        create option to wait further back from the backdrop and wait until all 3 april tags are detected to avoid robot collision
         */

        //Goal for comp 1: deposit both pixels on board + cycle once; feasible??




        if(audience){
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, mediaryTraj, centerTraj, travelTraj, leftBackdropTraj);
        } else {
            DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), spikeTraj, mediaryTraj, backdropTraj);
        }

        drive.followTrajectory(spikeTraj);
        drive.followTrajectory(mediaryTraj);
        if(!audience) {
            drive.followTrajectory(backdropTraj);
            while(aprilTagTranslations.get(audiencetag) == null){
                vision.update();
                telemetry.addLine("hello");
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(audiencetag, false);
            }
            telemetry.addLine("dog");
            telemetry.update();
            Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                    .strafeTo(aprilPoint.getPos())
                    .build();
            Trajectory tempParkTraj = drive.trajectoryBuilder(aprilTagTraj.end())
                .strafeTo(tempParkPoint.getPos())
                    .build();
            drive.followTrajectory(aprilTagTraj);
            //armServo.setPosition(0.275);
            sleep(2750);
            //armServo.setPosition(0.04);
            drive.followTrajectory(tempParkTraj);
        }
        if(audience){
            drive.followTrajectory(audienceTraj);
            drive.followTrajectory(centerTraj);
            drive.followTrajectory(travelTraj);
            drive.followTrajectory(leftBackdropTraj);
            while(aprilTagTranslations.get(audiencetag) == null){
                vision.update();
                telemetry.addLine("hello");
                telemetry.update();
                aprilTagTranslations = vision.getTranslationToTags();
                robotPose = vision.localize(audiencetag, false);
            }
            telemetry.addLine("dog");
            telemetry.update();
            Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                    .strafeTo(aprilPoint.getPos())
                    .build();
            Trajectory tempParkTrajLeft = drive.trajectoryBuilder(aprilTagTraj.end())
                    .strafeTo(tempParkPoint.getPos())
                    .build();
            drive.followTrajectory(aprilTagTraj);
            //armServo.setPosition(0.275);
            sleep(2750);
           // armServo.setPosition(0.04);
            drive.followTrajectory(tempParkTrajLeft);
        }
    }
}
