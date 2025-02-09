package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;


@Config
public class ArmPIDConstants {
    public static double ROTP = 0, ROTI = 0, ROTD = 0, ROTF = 0;
    public static double EXTP = 0, EXTI = 0, EXTD = 0, EXTF = 0;

    public static double rot_ticks_in_degree = 0.0417;
    public static double ext_ticks_in_degree = 0.0521;
}
