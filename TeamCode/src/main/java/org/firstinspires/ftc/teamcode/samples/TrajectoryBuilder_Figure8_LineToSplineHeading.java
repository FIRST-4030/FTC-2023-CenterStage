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
 * This is an example of drawing a figure 8 using strafeTo and lineTo trajectory modes
 */
@Disabled
@Autonomous(group = "drive")
public class TrajectoryBuilder_Figure8_LineToSplineHeading extends LinearOpMode {
    public static double HEIGHT = 60; // in
    public static double WIDTH = 30; // in

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(0, HEIGHT/2, Math.toRadians(180)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-WIDTH, HEIGHT/2, Math.toRadians(90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-WIDTH, HEIGHT, Math.toRadians(0)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(0, HEIGHT, Math.toRadians(270)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(0, HEIGHT/2, Math.toRadians(180)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-WIDTH, HEIGHT/2, Math.toRadians(270)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-WIDTH, 0, Math.toRadians(0)))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(90)))
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8 );

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
    }
}
