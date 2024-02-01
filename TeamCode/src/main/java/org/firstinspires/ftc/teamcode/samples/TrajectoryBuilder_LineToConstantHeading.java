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

/* Robot moves to the specified coordinates.
 * The robot maintains the heading it starts at throughout the trajectory
 * So, if you start at a 90 degree angle, it will keep that angle the entire path.
 *
 * Functionally the same as lineToConstantHeading() and lineTo()
 */
@Disabled
@Autonomous(group = "drive")
public class TrajectoryBuilder_LineToConstantHeading extends LinearOpMode {

    public static double X1 = 40; // in
    public static double X2 = 10; // in

    public static double Y1 = 30; // in
    public static double Y2 = 60; // in

    public Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(X1,Y1))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(X2,Y2))
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        drive.followTrajectory(traj2);
    }
}
