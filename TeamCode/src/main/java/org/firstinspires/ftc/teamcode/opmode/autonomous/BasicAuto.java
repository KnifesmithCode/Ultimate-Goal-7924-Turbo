package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.AccessoryPosition;
import org.firstinspires.ftc.teamcode.util.RingsPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Config
@Autonomous(name = "BasicAuto", group = "Autonomous")
public class BasicAuto extends LinearOpMode {
    public static boolean PREVIEW = true;

    public static boolean REV1 = true;
    public static boolean REV2 = false;
    public static boolean REV3 = false;
    public static boolean REV4 = false;
    public static boolean REV5 = false;

    private ElapsedTime runtime;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot(hardwareMap);
        } catch (LynxModuleUtil.LynxFirmwareVersionException e) {
            telemetry.addData("FirmwareErr", e.toString());
        }
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initializing camera...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        OpenCvCamera camera;
        if(PREVIEW) {
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

        while(!isStarted()) {
            telemetry.addData("qty", pipeline.getQty().name());
            telemetry.update();
        }

        telemetry.addData("Status", "Resetting encoders...");
        telemetry.update();

        robot.dt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dt.setPoseEstimate(PoseStorage.AUTO_POSE);

        TrajectoryBuilder builder1 = robot.dt.trajectoryBuilder(PoseStorage.AUTO_POSE, REV1);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        runtime.reset();

        Vector2d wobblePoint;
        switch(pipeline.getQty()) {
            default:
            case ZERO:
                // A
                wobblePoint = new Vector2d(10.0, 40.0);
                wobblePoint = new Vector2d(16.0, 40.0);
                break;
            case ONE:
                // B
                wobblePoint = new Vector2d(36.0, 16.0);
                wobblePoint = new Vector2d(42.0, 16.0);
                break;
            case FOUR:
                // C
                wobblePoint = new Vector2d(58.0, 40.0);
                wobblePoint = new Vector2d(64.0, 40.0);
                break;
        }

        robot.launcher.updateVelocity();

        Trajectory traj1 = robot.dt.trajectoryBuilder(PoseStorage.AUTO_POSE, true)
                .back(45)
                .build();

        Trajectory traj2 = robot.dt.trajectoryBuilder(traj1.end(), true)
                .back(40)
                .build();

        Trajectory traj3 = robot.dt.trajectoryBuilder(traj2.end(), true)
                .forward(35)
                .build();

        robot.dt.followTrajectory(traj1);
        for(int i = 0; i < 3; i++ ) {
            robot.launcher.setHammerPosition(AccessoryPosition.CLOSED);
            sleep(800);
            robot.launcher.setHammerPosition(AccessoryPosition.OPEN);
            sleep(800);
        }

        robot.wobbleArm.setArmPosition(AccessoryPosition.DOWN);

        robot.dt.followTrajectory(traj2);
        robot.wobbleLatch.toggleLatch();
        sleep(500);
        robot.dt.followTrajectory(traj3);

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
