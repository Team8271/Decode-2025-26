package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class Poses {

    public static class Blue {
        public static final Pose
                farStart        = new Pose(56,9,Math.PI/2),
                closeStart      = new Pose(17.29,116.89, Math.toRadians(143.19)),
                closeScore      = new Pose(42.8,92.2, Math.toRadians(129.5)),
                farScore        = new Pose(57.6,17.0, Math.toRadians(113.93)),
                lineUpSpike1    = new Pose(50,80.7, Math.PI),  // x was 42.7
                lineUpSpike2    = new Pose(50,57.9, Math.PI),
                lineUpSpike3    = new Pose(50,34.6, Math.PI),
                lineUpGate      = new Pose(23.5,58.0, Math.toRadians(139.16)),
                pickUpSpike1    = new Pose(18.5,80.7, Math.PI),
                pickUpSpike2    = new Pose(10.6,57.9, Math.PI),
                pickUpSpike3    = new Pose(11.7,34.6, Math.PI),
                exitSpike1      = new Pose(24,80.7,   Math.PI),
                exitSpike2      = new Pose(24,57.9, Math.PI),
                exitSpike3      = new Pose(24,34.6, Math.PI),
                exitGate        = lineUpGate,
                pickUpGate      = new Pose(13.34,57.85, Math.toRadians(141.45)),
                farPark         = new Pose(60.3,12.1, Math.PI/2),
                closeScorePark  = new Pose(51.8,102.3, Math.toRadians(141.84)),
                spike1Launch    = new Pose(59,78.5,Math.toRadians(132));
    }

    public static class Red {
        public static final Pose
                farStart        = new Pose(88,9,Math.PI/2),
                closeStart      = new Pose(),
                closeScore      = new Pose(),
                farScore        = new Pose(),
                lineUpSpike1    = new Pose(),
                lineUpSpike2    = new Pose(),
                lineUpSpike3    = new Pose(),
                lineUpGate      = new Pose(),
                pickUpSpike1    = new Pose(),
                pickUpSpike2    = new Pose(),
                pickUpSpike3    = new Pose(),
                exitSpike1      = new Pose(),
                exitSpike2      = new Pose(),
                exitSpike3      = new Pose(),
                exitGate        = new Pose(),
                pickUpGate      = new Pose(),
                farPark         = new Pose(),
                closeScorePark  = new Pose();
    }
}
