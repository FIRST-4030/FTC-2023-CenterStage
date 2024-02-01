package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.drives.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.general.Pose2dWrapper;

/*
 * This is an example of a forward/backward trajectory mode
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class TrajectoryBuilder_ForwardBackward extends LinearOpMode {
    public static double UP = 40; // in
    public static double BACK = 30; // in
    public static int PAUSE = 1000; // msec

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(UP)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .back(BACK)
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        sleep(PAUSE);

        drive.followTrajectory(traj2);
    }
}
