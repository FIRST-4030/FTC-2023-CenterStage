package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.OPENING_CAMERA_DEVICE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import org.firstinspires.ftc.teamcode.drive.drives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@Autonomous(name = "MecanumAuto")
public class MecanumAuto extends LinearOpMode {
    public int spike = 2;
    double spikePointX = 12;
    double spikePointY = -34.5;
    public  double spikeHeading = 90;
    public boolean audience = false;
    double backdropX = 50.75;
    int backdropCenterAT = 5;
    int audienceAT = 8;
    double trackWidth = TRACK_WIDTH;


    public Pose2dWrapper startPose = new Pose2dWrapper(15, -62.5, Math.toRadians(90));
    public Pose2dWrapper mediaryPose = new Pose2dWrapper(15, -50.5, 0);
    public Pose2dWrapper backdropPose = new Pose2dWrapper(33, -36.5, 0);
    public Pose2dWrapper centerPose = new Pose2dWrapper(-57, -10, 0);
    public Pose2dWrapper outerCenterPose = new Pose2dWrapper(-33, -58.5, 0);
    public Pose2dWrapper tempParkPose = new Pose2dWrapper(48, -61.5, 0);
    public Pose2dWrapper travelPose = new Pose2dWrapper(25, -10, 0);
    public Pose2dWrapper outerTravelPose = new Pose2dWrapper(24, -58.5, 0);
    public Pose2dWrapper aprilTagPose = new Pose2dWrapper(51.5, -37, 0);
    public Pose2dWrapper pixelPose = new Pose2dWrapper(-53, -37, 0);
    public Pose2dWrapper postPixelPose = new Pose2dWrapper(-59.25, -36.5, 0);
    public Pose2dWrapper avoidancePose = new Pose2dWrapper(-58 , -36.5, 0);
    public Pose2dWrapper secondCollectionPose = new Pose2dWrapper(-59.25, -10, 0);
    public Pose2dWrapper finalDepositPose = new Pose2dWrapper(52, -39, 0);
    public Pose2dWrapper preSecondCollectionPose = new Pose2dWrapper(-53, -10, 0);


    ComputerVision vision;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];
    InputHandler inputHandler;
    Servo armServo;

    boolean inputComplete = false;
    boolean isBlue = false;
    boolean parkInCenter = false;
    boolean travelUnderTruss = false;
    boolean collectWhitePixels = false;
    Pose2d robotPose;

    Servo leftFlipper;
    Servo rightFlipper;
    DcMotorSimple intake;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime failsafeTimer = new ElapsedTime();
    ElapsedTime doomsdayClock = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();
    int startDelay = 0;
    boolean catastrophicFailure = false;
    boolean failstate = false;
    int i = 1; //used as an iterator for outputLog()
    boolean returned = false;
    int pixelSide = 2;
    double pixelOffset = 0;
    String pixelSideInEnglish;
    Trajectory pixelTraj;
    Trajectory outerTravelTraj;
    Trajectory outerPixelTraj;
    Trajectory precisionCollectionTraj;
    SecondPixelDetector pixelSensor;



    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        inputHandler = InputAutoMapper.normal.autoMap(this);
        while(inputComplete == false){
            inputHandler.loop();
            if(inputTimer.milliseconds() > 250){
            if(inputHandler.up("D1:DPAD_LEFT")){
                isBlue = !isBlue;
                inputTimer.reset();
            }
            if(inputHandler.up("D1:DPAD_RIGHT")){
                audience = !audience;
                inputTimer.reset();
            }
            if(inputHandler.up("D1:DPAD_UP")){
                pixelSide++;
                inputTimer.reset();
            }
            if(inputHandler.up("D1:DPAD_DOWN")){
                parkInCenter = !parkInCenter;
                inputTimer.reset();
            }
            if(inputHandler.up("D1:LT")){
                startDelay += 1000;
                inputTimer.reset();
            }
            if(inputHandler.up("D1:Y")){
                travelUnderTruss = !travelUnderTruss;
                inputTimer.reset();
            }
            startDelay = startDelay % 6000;
            switch((pixelSide % 3) + 1){
                case 1:
                    pixelSideInEnglish = "Left";
                    pixelOffset = 3;
                    break;
                case 2:
                    pixelSideInEnglish = "Center";
                    pixelOffset = 0;
                    break;
                case 3:
                    pixelSideInEnglish = "Right";
                    pixelOffset = -2;
                    break;
                }
            if(inputHandler.up("D1:RT")){
                collectWhitePixels = !collectWhitePixels;
            }

            if(inputHandler.up("D1:X")){
                inputComplete = true;
                inputTimer.reset();
                }

            }
            telemetry.addData("-----Initialization-----", "");
            telemetry.addData("is Blue:", isBlue);
            telemetry.addData("is Near Audience:", audience);
            telemetry.addData("Travel Under Truss: ", travelUnderTruss);
            telemetry.addLine();
            telemetry.addData("-----Modifications-----", "");
            telemetry.addData("Yellow Pixel Deposit Offset: ", pixelSideInEnglish);
            telemetry.addData("Park in center: ", parkInCenter);
            telemetry.addData("Current delay: ", startDelay);
            telemetry.addLine();
            telemetry.addData("Press X to finalize values", inputComplete);
            if(!audience) {
                telemetry.addData("Collect 2 Whites: ", collectWhitePixels);
            }
            telemetry.update();


        }
        vision = new ComputerVision(hardwareMap);
        while(vision.visionPortal.getCameraState() == OPENING_CAMERA_DEVICE){

        }
        vision.setActiveCameraOne();
        armServo = hardwareMap.get(Servo.class, "Arm");
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        leftFlipper = hardwareMap.get(Servo.class, "leftHook");
        rightFlipper = hardwareMap.get(Servo.class, "rightHook");

        leftFlipper.setPosition(0.4);
        rightFlipper.setPosition(0.6);

        intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        pixelSensor = new SecondPixelDetector(hardwareMap);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0);

        do {
            vision.updateTensorFlow();
            spike = vision.checkSpike(isBlue, audience);
            telemetry.addData("spike: ", spike);
            telemetry.update();
        } while(opModeInInit());
        telemetry.addData("Started", " ");
        telemetry.update();


        if (isStopRequested()) return;
        if(isBlue) {
            switch (spike) {
                case 3:
                    spikePointX = 3;
                    spikePointY = -34;
                    spikeHeading = 115;
                    aprilTagPose.y = -28.3 + pixelOffset;
                    break;
                case 2:
                    spikePointX = 9.5;
                    spikePointY = -33;
                    spikeHeading = 105;
                    aprilTagPose.y = -35.5 + pixelOffset;
                    break;
                case 1:
                    spikePointX = 17.5;
                    spikePointY = -33.5;
                    spikeHeading = 65;
                    aprilTagPose.y = -42.4 + pixelOffset;
                    break;
            }
        } else {
            switch (spike) {
                case 1:
                    spikePointX = 3;
                    spikePointY = -34;
                    spikeHeading = 115;
                    aprilTagPose.y = -28.3 + pixelOffset;
                    break;
                case 2:
                    spikePointX = 9.5;
                    spikePointY = -33;
                    spikeHeading = 105;
                    aprilTagPose.y = -35.5 + pixelOffset;
                    break;
                case 3:
                    spikePointX = 17.5;
                    spikePointY = -33.5;
                    spikeHeading = 65;
                    aprilTagPose.y = -42.4 + pixelOffset;
                    break;
            }
        }
        telemetry.addData("Current Y:", aprilTagPose.y);
        telemetry.update();

        Pose2dWrapper spikePose = new Pose2dWrapper(spikePointX, spikePointY, spikeHeading);


        if(audience){
            startPose.x = -39;
            backdropX = 47.75;

            spikePose.x -= 46;
            if(spike == 2){
                spikePose.x += 4;
                spikePose.heading = 75;
            }
            mediaryPose.x = -(mediaryPose.x + 34);
        }
        if(parkInCenter){
            tempParkPose.y = -10;
        }
        if(isBlue){
            startPose.y *= -1;
            startPose.heading *= -1;
            audienceAT = 9;
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
            travelPose.y *= -1;
            travelPose.heading *= -1;
            tempParkPose.y *= -1;
            tempParkPose.heading *= -1;
            aprilTagPose.y *= -1;
            aprilTagPose.heading *= -1;
            pixelPose.y *= -1;
            pixelPose.heading *= -1;
            postPixelPose.y *= -1;
            postPixelPose.heading *= -1;
            outerTravelPose.y *= -1;
            outerTravelPose.heading *= -1;
            outerCenterPose.y *= -1;
            outerCenterPose.heading *= -1;
            secondCollectionPose.y *= -1;
            secondCollectionPose.heading *= -1;
            finalDepositPose.y *= -1;
            finalDepositPose.heading *= -1;
            preSecondCollectionPose.y *= -1;
            preSecondCollectionPose.heading *= -1;
        }

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory spikeTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(spikePose.toPose2d().vec(), Math.toRadians(spikePose.heading))
                .build();

        telemetry.addData("Started Running", " ");
        telemetry.update();
        sleep(startDelay);
        outputLog(drive); //1
        drive.followTrajectory(spikeTraj);
        if(!audience) {
            outputLog(drive); //2
            Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                    .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180-mediaryPose.heading))
                    .build();
            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            Trajectory backdropTraj = drive.trajectoryBuilder(mediaryTraj.end())
                    .strafeTo(backdropPose.toPose2d().vec())
                    .build();
            drive.followTrajectory(backdropTraj);
            outerTravelTraj = depositPixel(drive, !collectWhitePixels);
            vision.setActiveCameraTwo();
            if(catastrophicFailure == true || !collectWhitePixels){
                drive.followTrajectory(outerTravelTraj);
                Trajectory finalParkTraj = drive.trajectoryBuilder(outerTravelTraj.end())
                        .strafeTo(tempParkPose.toPose2d().vec())
                        .build();
                drive.followTrajectory(finalParkTraj);
                return;
            } else {
                Trajectory outerCenterTraj = drive.trajectoryBuilder(outerTravelTraj.end())
                        .strafeTo(outerCenterPose.toPose2d().vec())
                        .build();
                Trajectory outerPixelTraj = drive.trajectoryBuilder(outerCenterTraj.end())
                        .strafeTo(pixelPose.toPose2d().vec())
                        .build();
                drive.followTrajectory(outerTravelTraj);
                drive.followTrajectory(outerCenterTraj);
                drive.followTrajectory(outerPixelTraj);
                outputLog(drive);
                vision.tensorFlowProcessor.shutdown();
                Trajectory returnTraj = collectPixelAudience(drive, 2);
                drive.followTrajectory(returnTraj);
                outputLog(drive);
                Trajectory parkTraj = depositPixel(drive, true);
                if (catastrophicFailure == true) {
                    drive.followTrajectory(outerTravelTraj);
                    Trajectory finalParkTraj = drive.trajectoryBuilder(outerTravelTraj.end())
                            .strafeTo(tempParkPose.toPose2d().vec())
                            .build();
                    drive.followTrajectory(finalParkTraj);
                    return;
                }
                drive.followTrajectory(parkTraj);
                sleep(1000);
            }

        }


        if(audience){
            outputLog(drive); //2
            vision.setActiveCameraTwo();
            Trajectory mediaryTraj = drive.trajectoryBuilder(spikeTraj.end(), true)
                    .splineTo(mediaryPose.toPose2d().vec(), Math.toRadians(180-mediaryPose.heading))
                    .build();
            drive.followTrajectory(mediaryTraj);
            outputLog(drive); //3
            pixelTraj = drive.trajectoryBuilder(mediaryTraj.end())
                    .strafeTo(pixelPose.toPose2d().vec())
                    .build();
            drive.followTrajectory(pixelTraj);
            outputLog(drive); //4
            vision.tensorFlowProcessor.shutdown();
            Trajectory avoidanceTraj = collectPixelAudience(drive, 1);
            Trajectory centerTraj = drive.trajectoryBuilder(avoidanceTraj.end())
                    .strafeTo(centerPose.toPose2d().vec())
                            .build();
            Trajectory travelTraj = drive.trajectoryBuilder(centerTraj.end())
                    .splineTo(travelPose.toPose2d().vec(), Math.toRadians(travelPose.heading),
                            NewMecanumDrive.getVelocityConstraint(60, 1.55, 13.45),
                            NewMecanumDrive.getAccelerationConstraint(60))
                            .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading),
                                    NewMecanumDrive.getVelocityConstraint(50, 1.55, 13.45),
                                    NewMecanumDrive.getAccelerationConstraint(50))
                            .build();
            drive.followTrajectory(avoidanceTraj);
            outputLog(drive);
            if(!travelUnderTruss){
            drive.followTrajectory(centerTraj);
            outputLog(drive);
            drive.followTrajectory(travelTraj);
            outputLog(drive);
            }
            Trajectory parkTraj = depositPixel(drive, true);
            //drive.followTrajectory(parkTraj);
            sleep(1000);
            return;

        }
    }
    public Trajectory depositPixel(NewMecanumDrive drive, boolean fin){
        Trajectory tempTrajDeposit;
        intake.setPower(0);
        vision.setActiveCameraOne();
        doomsdayClock.reset();
        while(aprilTagTranslations[backdropCenterAT] == null && doomsdayClock.milliseconds() < 7500){
            vision.updateAprilTags();
            aprilTagTranslations = vision.getTranslationToTags();
            robotPose = vision.localize(backdropCenterAT, true);
            if(doomsdayClock.milliseconds() > 2000){
                armServo.setPosition(0.04);
            }
        }
        if(doomsdayClock.milliseconds() > 7500){
            catastrophicFailure = true;
            if(audience) {
                tempTrajDeposit = drive.trajectoryBuilder(backdropPose.toPose2d())
                        .splineToConstantHeading(travelPose.toPose2d().vec(), travelPose.heading)
                        .splineToConstantHeading(tempParkPose.toPose2d().vec(), travelPose.heading)
                        .build();
            } else {
                tempTrajDeposit = drive.trajectoryBuilder(backdropPose.toPose2d())
                        .splineToConstantHeading(outerTravelPose.toPose2d().vec(), travelPose.heading)
                        .splineToConstantHeading(tempParkPose.toPose2d().vec(), travelPose.heading)
                        .build();
            }
            tempTrajDeposit = drive.trajectoryBuilder(aprilTagPose.toPose2d())
                    .splineToConstantHeading(travelPose.toPose2d().vec(), travelPose.heading)
                    .splineToConstantHeading(tempParkPose.toPose2d().vec(), travelPose.heading)
                    .build();
            return tempTrajDeposit;
        }
        drive.setPoseEstimate(robotPose);
        outputLog(drive); //9
        Trajectory aprilTagTraj = drive.trajectoryBuilder(robotPose)
                .strafeTo(aprilTagPose.toPose2d().vec(),
                        NewMecanumDrive.getVelocityConstraint(30, 1.55, trackWidth),
                        NewMecanumDrive.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(aprilTagTraj);
        sleep(500);
        armServo.setPosition(0.285);
        outputLog(drive); //10
        sleep(2500);
        armServo.setPosition(0.04);
        if(!audience) {
            sleep(1000);
        }
        if(!fin) {
            telemetry.addData(aprilTagTraj.end().toString(), "");
            telemetry.update();
            tempTrajDeposit = drive.trajectoryBuilder(aprilTagTraj.end(), true)
                    .strafeTo(outerTravelPose.toPose2d().vec())
                    .build();
        }
        else {
                tempTrajDeposit = drive.trajectoryBuilder(aprilTagTraj.end())
                        .back(4)
                        .build();

        }

        return tempTrajDeposit;
    }

    public Trajectory collectPixelAudience(NewMecanumDrive drive, int numPixels){
        Trajectory centerTraj;
        outputLog(drive); //5
        failsafeTimer.reset();
        while(aprilTagTranslations[audienceAT] == null){
            if(failsafeTimer.milliseconds() > 3000){
                if (audience) {
                    centerTraj = drive.trajectoryBuilder(pixelTraj.end())
                            .strafeTo(avoidancePose.toPose2d().vec())
                            .build();
                } else {
                    centerTraj = drive.trajectoryBuilder(outerPixelTraj.end())
                            .splineToConstantHeading(outerCenterPose.toPose2d().vec(), Math.toRadians(outerCenterPose.heading))
                            .splineToConstantHeading(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
                            .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                            .build();
                }
                return centerTraj;
            }
            vision.updateAprilTags();
            aprilTagTranslations = vision.getTranslationToTags();
            robotPose = vision.localize(audienceAT, false);
        }
        drive.setPoseEstimate(robotPose);
        outputLog(drive); //6
        Trajectory postPixelTraj =  drive.trajectoryBuilder(robotPose)
                .lineToConstantHeading(postPixelPose.toPose2d().vec(),
                        NewMecanumDrive.getVelocityConstraint(30, 1.85, trackWidth),
                        NewMecanumDrive.getAccelerationConstraint(30))
                .build();
        if (audience && !travelUnderTruss) {
            centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .strafeTo(avoidancePose.toPose2d().vec())
                    .build();
        } else {
            centerTraj = drive.trajectoryBuilder(postPixelTraj.end())
                    .splineToConstantHeading(outerCenterPose.toPose2d().vec(), Math.toRadians(outerCenterPose.heading))
                    .splineToConstantHeading(outerTravelPose.toPose2d().vec(), Math.toRadians(outerTravelPose.heading))
                    .splineToConstantHeading(backdropPose.toPose2d().vec(), Math.toRadians(backdropPose.heading))
                    .build();
        }


        drive.followTrajectory(postPixelTraj);
        outputLog(drive);//7
        for(int i = 0; i < numPixels; i++ ) {
            intake.setPower(1);
            leftFlipper.setPosition(0.999);
            rightFlipper.setPosition(0.001);
            sleep(200);
            leftFlipper.setPosition(0.4);
            rightFlipper.setPosition(0.6);
            if(i+1 < numPixels){
                sleep(500);
            }
        }
        return centerTraj;
    }
    public void outputLog(NewMecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()), runtime.milliseconds(), i);
        i++;
    }
}
