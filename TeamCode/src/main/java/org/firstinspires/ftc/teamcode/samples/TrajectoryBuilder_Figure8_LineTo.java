package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class TrajectoryBuilder_Figure8_LineTo extends LinearOpMode {
    public static double HEIGHT = 60; // in
    public static double WIDTH = 30; // in

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, HEIGHT/2))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-WIDTH, HEIGHT/2))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-WIDTH, HEIGHT))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(0, HEIGHT))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(0, HEIGHT/2))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(-WIDTH, HEIGHT/2))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(-WIDTH, 0))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineTo(new Vector2d(0, 0))
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
