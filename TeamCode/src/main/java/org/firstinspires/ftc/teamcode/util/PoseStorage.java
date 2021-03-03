package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class PoseStorage {
    public static Pose2d POSE = new Pose2d(-63.0, 40.0, Math.toRadians(0.0));
    public static final Pose2d AUTO_POSE = new Pose2d(-63.0, 40.0, Math.toRadians(180.0));

    public static final Pose2d SHOOTING_POSE = new Pose2d(-2.0, 32.0, Math.toRadians(180.0));

    public static final Pose2d POWERSHOT_POSE = new Pose2d(0.0, -15.0, Math.toRadians(180.0));
}
