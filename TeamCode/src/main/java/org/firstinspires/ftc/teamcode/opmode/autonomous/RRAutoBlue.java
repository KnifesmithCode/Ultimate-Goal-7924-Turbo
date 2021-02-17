package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.AccessoryPosition;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RingsPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Config
@Autonomous(name = "RR Auto Blue", group = "Autonomous")
@SuppressWarnings("unused")
public class RRAutoBlue extends LinearOpMode {
    public static boolean PREVIEW = true;

    private Robot robot;

    @Override
    public void runOpMode() {
        ElapsedTime runtime;

        try {
            robot = new Robot(hardwareMap);
        } catch (LynxModuleUtil.LynxFirmwareVersionException e) {
            telemetry.addData("FirmwareErr", e.toString());
        }
        runtime = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initializing camera...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        OpenCvCamera camera;
        if (PREVIEW) {
            camera = OpenCvCameraFactory.getInstance()
                    .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
            camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        } else {
            camera = OpenCvCameraFactory.getInstance()
                    .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
        }

        camera.openCameraDevice();
        camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);

        RingsPipeline pipeline = new RingsPipeline();
        camera.setPipeline(pipeline);

        robot.wobbleLatch.setLatchPosition(AccessoryPosition.CLOSED);

        while (!isStarted()) {
            telemetry.addData("qty", pipeline.getQty().name());
            telemetry.update();
        }

        telemetry.addData("Status", "Resetting encoders...");
        telemetry.update();

        robot.dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dt.setPoseEstimate(PoseStorage.AUTO_POSE);

        // First wobble goal to correct position
        TrajectoryBuilder builder1 =
                robot.dt.trajectoryBuilder(
                        PoseStorage.AUTO_POSE, Math.toRadians(PoseStorage.AUTO_POSE.getHeading()));

        telemetry.addData("Status", "Ready");
        telemetry.update();

        runtime.reset();

        double wobbleY = 52.0;
        Vector2d wobblePoint;
        switch (pipeline.getQty()) {
            default:
            case ZERO:
                // A
                wobblePoint = new Vector2d(16.0, wobbleY);
                break;
            case ONE:
                // B
                wobblePoint = new Vector2d(40.0, wobbleY - 24.0);
                break;
            case FOUR:
                // C
                wobblePoint = new Vector2d(60.0, wobbleY);
                break;
        }

        telemetry.addData("Status", "Planning paths...");
        telemetry.update();

        robot.launcher.updateVelocity();
        builder1.splineTo(wobblePoint, Math.toRadians(0.0));
        Trajectory traj1 = builder1.build();

        // Get to shooting position
        Trajectory traj2 = robot.dt.trajectoryBuilder(traj1.end(), Math.toRadians(180.0))
                .splineTo(new Vector2d(-8.0, 34.0), Math.toRadians(180.0))
                .build();

        // Go to grab second wobble goal
        Trajectory traj3 = robot.dt.trajectoryBuilder(traj2.end(), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(-24.0, 24.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                .splineToConstantHeading(new Vector2d(-32.0, 24.0), Math.toRadians(-90.0))
                .build();

        Vector2d secondWobblePoint = new Vector2d(wobblePoint.getX() - 8.0, wobblePoint.getY() - 8.0);

        // Drop off second wobble goal
        Trajectory traj4 = robot.dt.trajectoryBuilder(traj3.end(), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(secondWobblePoint, Math.toRadians(-180.0)), Math.toRadians(-180.0))
                .build();

        // Park on line
        Trajectory traj5 = robot.dt.trajectoryBuilder(traj4.end(), Math.toRadians(-180.0))
                .splineToSplineHeading(new Pose2d(10.0, 16.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        telemetry.addData("Status", "Following traj1");
        telemetry.update();

        robot.dt.followTrajectory(traj1);

        robot.wobbleLatch.toggleLatch();
        sleep(100);

        telemetry.addData("Status", "Following traj2");
        telemetry.update();

        robot.dt.followTrajectory(traj2);

        telemetry.addData("Status", "Shooting");
        telemetry.update();

        for (int i = 0; i < 3; i++) {
            robot.launcher.setHammerPosition(AccessoryPosition.ENGAGED);
            sleep(750);
            robot.launcher.setHammerPosition(AccessoryPosition.DISENGAGED);
            if (i < 2) {
                sleep(2000);
            }
        }

        robot.wobbleArm.setArmPosition(AccessoryPosition.DOWN);
        robot.wobbleArm.setClawPosition(AccessoryPosition.OPEN);

        telemetry.addData("Status", "Following traj3");
        telemetry.update();

        robot.dt.followTrajectory(traj3);

        robot.wobbleArm.setClawPosition(AccessoryPosition.CLOSED);
        sleep(1000);
        robot.wobbleArm.setArmPosition(AccessoryPosition.MIDDLE);
        sleep(500);

        telemetry.addData("Status", "Following traj4");
        telemetry.update();

        robot.dt.followTrajectory(traj4);

        robot.wobbleArm.setArmPosition(AccessoryPosition.DOWN);
        sleep(500);
        robot.wobbleArm.setClawPosition(AccessoryPosition.OPEN);
        sleep(1000);
        robot.wobbleArm.setArmPosition(AccessoryPosition.MIDDLE);
        while (robot.wobbleArm.armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Status", "Waiting for arm");
            telemetry.update();
        }

        telemetry.addData("Status", "Following traj5");
        telemetry.update();

        robot.dt.followTrajectory(traj5);

        robot.wobbleArm.setArmPosition(AccessoryPosition.UP);
        while (robot.wobbleArm.armMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Status", "Waiting for arm");
            telemetry.update();
        }

        PoseStorage.POSE = robot.dt.getPoseEstimate();
        stop();
    }

    private void launchRing(ElapsedTime runtime) {
        double initial = runtime.milliseconds();
        while (runtime.milliseconds() < (initial + 1000d) && opModeIsActive()) {
            robot.launcher.setHammerPosition(AccessoryPosition.CLOSED);
        }
        while (runtime.milliseconds() < (initial + 2000d) && opModeIsActive()) {
            robot.launcher.setHammerPosition(AccessoryPosition.OPEN);
        }
    }


}
