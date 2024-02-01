package org.firstinspires.ftc.teamcode.robot2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

@Disabled
@TeleOp
public class VisionOpMode2 extends OpMode {
    public ComputerVision2 computerVision;
    boolean isBlue = false;
    AprilTagPoseFtc aprilTagFivePose;
    @Override
    public void init() {
        computerVision = new ComputerVision2(hardwareMap);
    }

    @Override
    public void loop() {
        computerVision.update();
        //telemetryTfod();
        telemetry.addData("Spike: ", computerVision.checkSpike(isBlue));
        aprilTagFivePose = computerVision.getTranslationToTags().get(5);
        Pose2d robotLocation = computerVision.localize(5, false);
        if(aprilTagFivePose != null) {
            telemetry.addData("X Translation: ", aprilTagFivePose.x);
            telemetry.addData("y Translation: ", aprilTagFivePose.y);
        }
        telemetry.addData("robot X: ", robotLocation.getX());
        telemetry.addData("robot Y: ", robotLocation.getY());
        telemetry.addData("robot Heading: ", robotLocation.getHeading());
        if(computerVision.getTensorFlowRecognitions().size() > 0) {
            telemetry.addData("Left side: ", computerVision.getTensorFlowRecognitions().get(0).getLeft());
        }
        for (AprilTagDetection detection : computerVision.aprilTagDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
       // telemetry.addData("TF Recognitions: ", + computerVision.getTensorFlowRecognitions().size());
       // if(computerVision.getTensorFlowRecognitions().size() > 0) {
      //      telemetry.addData("left pos: ", computerVision.getTensorFlowRecognitions().get(0).getLeft());
      //  }
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = computerVision.getTensorFlowRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}
