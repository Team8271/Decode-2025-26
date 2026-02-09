package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class Poses {

    public static class Blue {
        public static final Pose
                farStart = new Pose(56, 9, Math.PI / 2),
                closeStart = new Pose(20.2, 118.37, Math.toRadians(141.90)),
                closeScore = new Pose(55.6, 78.8, Math.toRadians(131.65)),
                farScore = new Pose(57.6, 17.0, Math.toRadians(113.93)),
                lineUpSpike1 = new Pose(50, 80.7, Math.PI),  // x was 42.7
                lineUpSpike2 = new Pose(50, 57.9, Math.PI),
                lineUpSpike3 = new Pose(50, 34.6, Math.PI),
                lineUpGate = new Pose(23.5, 58.0, Math.toRadians(139.16)),
                pickUpSpike1 = new Pose(18.5, 80.7, Math.PI),
                pickUpSpike2 = new Pose(10.6, 57.9, Math.PI),
                pickUpSpike3 = new Pose(11.7, 34.6, Math.PI),
                exitSpike1 = new Pose(45, 80.7, Math.PI),
                exitSpike2 = new Pose(45, 57.9, Math.PI),
                exitSpike3 = new Pose(45, 34.6, Math.PI),
                exitGate = lineUpGate,
                openGate = new Pose(16.4, 63.83, Math.toRadians(270)),
                pickUpGate = new Pose(13.34, 59.4, Math.toRadians(141.45)),
                farPark = new Pose(60.3, 12.1, Math.PI / 2),
                closeScorePark = new Pose(59.4, 105.9, Math.toRadians(149.2)),
                spike1Launch = new Pose(59, 78.5, Math.toRadians(132));
    }

    public static class Red {
        public static final Pose
                farStart = new Pose(88, 9, Math.PI / 2),
                closeStart = new Pose(125.8, 118.0, Math.toRadians(37.64)),
                closeScore = new Pose(87.1, 76.4, Math.toRadians(48.3)),
                farScore = new Pose(91.1, 11.9, Math.toRadians(69.73)),
                lineUpSpike1 = new Pose(102.4, 81.0, 0),
                lineUpSpike2 = new Pose(101.0, 57.4, 0),
                lineUpSpike3 = new Pose(101.1, 33.9, 0),
                lineUpGate = new Pose(121.1, 56.6, Math.toRadians(30.9)),
                pickUpSpike1 = new Pose(124.8, 81.0, 0),
                pickUpSpike2 = new Pose(134.0, 56.5, 0),
                pickUpSpike3 = new Pose(133.0, 33.7, 0),
                exitSpike1 = lineUpSpike1,
                exitSpike2 = lineUpSpike2,
                exitSpike3 = lineUpSpike3,
                exitGate = lineUpGate,
                openGate = new Pose(127.2, 63.0, Math.toRadians(-90)),
                pickUpGate = new Pose(130.5, 57.4, Math.toRadians(33.0)),
                farPark = new Pose(111.8, 12.3, Math.toRadians(90)),
                closeScorePark = new Pose(88.8, 102.69, Math.toRadians(34.38));
    }
}
