package org.firstinspires.ftc.teamcode.util.tuners;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.AimAssist;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.Poses;

@TeleOp(name="HeadingPID TeleOp", group="Drive")
public class HeadingPIDTuner extends LinearOpMode {
    private Follower follower;

    Config robot;
    Pose startingPose;
    ElapsedTime runtime = new ElapsedTime();
    double lastAACalcTime = 0;

    private final Pose startPose = new Pose(88, 12, Math.toRadians(90)); // Start Pose of robot.


    private double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Config(this, null);
        robot.init();

        robot.setAlliance(Config.Alliance.BLUE);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.Blue.farStart);
        follower.update();

        //double headingCalc = robot.aimAssist.getHeadingForTarget(follower.getPose(),robot.alliance.getPose());
        double headingCalc = follower.getHeading();


        HeadingPIDF headingPID = new HeadingPIDF(1.5, 0.0, 0.0,0.01);

        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        targetHeading = getHeading();
        boolean throttleEffectsYaw = true;


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
                throttleEffectsYaw = true;
            }
            else {
                // Run intensive calculations every delay seconds
                double delay = 20;
                if (runtime.milliseconds() > lastAACalcTime + delay) {

                    double error = follower.getHeading()-headingCalc;

                    //telemetry.addData("AA ERROR", error);

                    yawControl = robot.aimAssist.headingPIDF.calculate(error);

                    throttleEffectsYaw = false;

                    lastAACalcTime = runtime.milliseconds();
                }
            }

            setTeleOpDrive(axial, lateral, yawControl, throttle, throttleEffectsYaw);


            // ---- PID Tuning via Gamepad ----
            if (gamepad1.dpad_up)    headingPID.kP += 0.01;
            if (gamepad1.dpad_down)  headingPID.kP -= 0.01;
            if (gamepad1.dpad_right) headingPID.kD += 0.005;
            if (gamepad1.dpad_left)  headingPID.kD -= 0.005;
            if (gamepad1.y)          headingPID.kI += 0.001;
            if (gamepad1.a)          headingPID.kI -= 0.001;
            if (gamepad2.dpadUpWasPressed())    headingPID.kF += 0.01;
            if (gamepad2.dpadDownWasPressed())    headingPID.kF -= 0.01;


            telemetry.addData("kP", headingPID.kP);
            telemetry.addData("kI", headingPID.kI);
            telemetry.addData("kD", headingPID.kD);
            telemetry.addData("kF", headingPID.kF);
            telemetry.addData("Yaw Correction", yawControl);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", getHeading());
            telemetry.update();
            sleep(1);
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

    public class HeadingPIDF {

        private double kP;
        private double kI;
        private double kD;
        private double kF; // feedforward gain

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private long lastTime;

        private double minOutput = -1.0;
        private double maxOutput = 1.0;

        public HeadingPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            lastTime = System.nanoTime();
        }

        public void setOutputLimits(double max) {
            minOutput = -max;
            maxOutput = max;
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime();
        }

        /**
         * @param error Distance in radians from target heading (AutoWrapped)
         * @return A value between -1 and 1 used for yaw motions
         */
        public double calculate(double error) {

            error = AimAssist.angleWrap(error);

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            // PID terms
            integralSum += error * deltaTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0.0;
            lastError = error;

            double pTerm = kP * error;
            double iTerm = kI * integralSum;
            double dTerm = kD * derivative;

            // Feedforward (static friction compensation)
            double fTerm = kF * Math.signum(error);

            double output = pTerm + iTerm + dTerm + fTerm;

            // Clamp output
            if (output > maxOutput) output = maxOutput;
            if (output < minOutput) output = minOutput;

            return output;
        }
    }

}
