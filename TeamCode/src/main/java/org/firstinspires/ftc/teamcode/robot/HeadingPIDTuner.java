package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="HeadingPID TeleOp", group="Drive")
public class HeadingPIDTuner extends LinearOpMode {
    private Follower follower;

    Config robot;
    Pose startingPose;

    private final Pose startPose = new Pose(88, 12, Math.toRadians(90)); // Start Pose of robot.


    private double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Config(this, null);
        robot.init();

        robot.setAlliance(Config.Alliance.BLUE);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        Pose targetGoal = new Pose(144,144);

        HeadingPID headingPID = new HeadingPID(1.5, 0.0, 0.0);
        headingPID.setOutputLimits(-0.6, 0.6);

        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        targetHeading = getHeading();

        while (opModeIsActive()) {
            follower.update();

            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double throttle = .2+(gamepad1.right_trigger*0.8); // throttle

            double gamepadRadians = Math.atan2(lateralControl, axialControl);
            double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
            double robotRadians = getHeading();
            double targetRadians = gamepadRadians + robotRadians;
            double lateral = Math.sin(targetRadians)*gamepadHypot;
            double axial = Math.cos(targetRadians)*gamepadHypot;

            // Manual yaw override
            if (Math.abs(yawControl) > 0.05) {
                targetHeading = getHeading();
                headingPID.reset();
            }
            else {
                // PID yaw correction
                double headingCalc = robot.aimAssist.getHeadingForTarget(follower.getPose(),targetGoal);

                telemetry.addData("Current Head",follower.getHeading());
                telemetry.addData("Heading Calc",headingCalc);
                telemetry.addData("Difference  ",follower.getHeading()-headingCalc);

                double error = follower.getHeading()-headingCalc;

                yawControl = headingPID.calculate(error);
            }

            // Drive
            setTeleOpDrive(axial, lateral, yawControl, throttle, false);

            // ---- PID Tuning via Gamepad ----
            if (gamepad1.dpad_up)    headingPID.kP += 0.001;
            if (gamepad1.dpad_down)  headingPID.kP -= 0.001;
            if (gamepad1.dpad_right) headingPID.kD += 0.0005;
            if (gamepad1.dpad_left)  headingPID.kD -= 0.0005;
            if (gamepad1.y)          headingPID.kI += 0.0001;
            if (gamepad1.a)          headingPID.kI -= 0.0001;

            telemetry.addData("kP", headingPID.kP);
            telemetry.addData("kI", headingPID.kI);
            telemetry.addData("kD", headingPID.kD);
            telemetry.addData("Yaw Correction", yawControl);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", getHeading());
            telemetry.update();
        }
    }

    private double getHeading() {
        return follower.getHeading();
    }

    private void setTeleOpDrive(double axial, double lateral, double yaw, double throttle, boolean throttleEffectsYaw) {
        double axialScaled = axial * throttle;
        double lateralScaled = lateral * throttle;
        double yawScaled = throttleEffectsYaw ? yaw * throttle : yaw;

        double leftFrontPower  = axialScaled + lateralScaled + yawScaled;
        double rightFrontPower = axialScaled - lateralScaled - yawScaled;
        double leftBackPower   = axialScaled - lateralScaled + yawScaled;
        double rightBackPower  = axialScaled + lateralScaled - yawScaled;

        double max = Math.max(1.0,
                Math.max(Math.abs(leftFrontPower),
                        Math.max(Math.abs(rightFrontPower),
                                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        robot.fl.setPower(leftFrontPower / max);
        robot.fr.setPower(rightFrontPower / max);
        robot.bl.setPower(leftBackPower / max);
        robot.br.setPower(rightBackPower / max);
    }

    class HeadingPID {

        private double kP;
        private double kI;
        private double kD;

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private long lastTime;

        private double minOutput = -1.0;
        private double maxOutput = 1.0;

        public HeadingPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            lastTime = System.nanoTime();
        }

        public void setOutputLimits(double min, double max) {
            minOutput = min;
            maxOutput = max;
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime();
        }

        /**
         *
         * @param error Distance in radians from target heading (AutoWrapped)
         * @return A value between -1 and 1 used for yaw motions
         */
        public double calculate(double error) {

            error = AimAssist.angleWrap(error);

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            integralSum += error * deltaTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0.0;
            lastError = error;

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);

            if (output > maxOutput) output = maxOutput;
            if (output < minOutput) output = minOutput;

            return output;
        }
    }
}
