package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.drives.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;

/*
 * Robot moves to the specified coordinates in a spline path
 * while separately linearly interpolating the heading
 *
 * The heading interpolates to the heading specified in `endPose`.
 * Setting `endTangent` affects the shape of the spline path itself.
 *
 * Due to the holonomic nature of mecanum drives, the bot is able
 * to make such a movement while independently controlling heading.
 *
 * 🚨  Will cause PathContinuityException's!! 🚨
 * Use splineToSplineHeading() if you are chaining these calls
 */
@Disabled
@Autonomous(group = "drive")
public class TrajectoryBuilder_SplineToSplineHeading extends LinearOpMode {
    public static double END_X = 40; // in
    public static double END_Y = 50; // in
    public static double HEADING1 = 45; // in
    public static double HEADING2 = 90; // in

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d( END_X, END_Y, Math.toRadians(HEADING1)), Math.toRadians(HEADING2))
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
    }
}
