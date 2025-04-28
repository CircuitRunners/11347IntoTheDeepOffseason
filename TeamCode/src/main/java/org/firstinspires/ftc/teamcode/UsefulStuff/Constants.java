package org.firstinspires.ftc.teamcode.UsefulStuff;

import com.acmerobotics.dashboard.config.Config;


@Config
public class Constants {
    public static double ROTP = 0, ROTI = 0, ROTD = 0, ROTF = 0;
    public static double EXTP = 0, EXTI = 0, EXTD = 0, EXTF = 0;

    //Pedro tuning TODO: actually measure if bad
    public static double mass = 15.5;

    public static double xMovement = 68.31575;
    public static double yMovement = 50.671225;

    public static double forwardZeroPowerAcceleration = -44.435875;
    public static double lateralZeroPowerAcceleration = -82.086;

    public static double rot_ticks_in_degree = 0.0417;
    public static double ext_ticks_in_degree = 0.0521;

    public static double pinpointXOffset = 1.41732; // MM = -36.0
    public static double pinpointYOffset = -3.18208661; // MM = -80.835 // 3.18208661
}
