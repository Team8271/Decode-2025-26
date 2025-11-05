package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.config.Selector;

import java.util.Map;

@TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {
    @Override
    public void runOpMode() {
        Config robot = new Config(this);
        robot.init();
        boolean debounce = false;
        boolean launcherDebounce = false;
        boolean selectorDebounce = false;
        double agitatorPower = 0;
        double intakeServoTarget = 0.5;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        robot.kickerMotor.setPower(robot.kickerIdlePower);

        while (opModeIsActive()) {
            // Driver 1 Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset

            // Driver 2 Controls
            boolean launchOneArtifact = gamepad2.a;
            boolean launchTwoArtifacts = gamepad2.y;
            boolean activateAgitatorAssembly = gamepad2.b;

            boolean increaseLauncherPower = gamepad2.dpad_up;
            boolean decreaseLauncherPower = gamepad2.dpad_down;


            if (launchOneArtifact) {
                robot.launcherThread.launch(1);
            }
            if (launchTwoArtifacts) {
                robot.launcherThread.launch(3);
            }

            // Dev launcher adjustments, to be handled with a limelight blackbox
            if (increaseLauncherPower && !launcherDebounce) {
                robot.increaseLauncherPower();
                launcherDebounce = true;
            }
            if (decreaseLauncherPower && !launcherDebounce) {
                robot.decreaseLauncherPower();
                launcherDebounce = true;
            }
            if (!increaseLauncherPower && !decreaseLauncherPower && launcherDebounce) {
                debounce = false;
            }


            if (activateAgitatorAssembly && !debounce) {
                if (agitatorPower == 1) {
                    agitatorPower = 0;
                    intakeServoTarget = 0.5;
                }
                else {
                    agitatorPower = 1;
                    intakeServoTarget = 1;
                }
                debounce = true;
            }


            if (!activateAgitatorAssembly && debounce) {
                debounce = false;
            }

            robot.agitator.setPower(agitatorPower);
            robot.intakeServo.setPosition(intakeServoTarget);


            // FCD reset
            if (resetFCD) {
                robot.odometer.resetTo(0,0,0);
            }

            // Calculate drive train power for field centric
            double gamepadRadians = Math.atan2(lateralControl, axialControl);
            double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
            double robotRadians = -robot.odometer.getZ();
            double targetRadians = gamepadRadians + robotRadians;
            double lateral = Math.sin(targetRadians)*gamepadHypot;
            double axial = Math.cos(targetRadians)*gamepadHypot;

            double leftFrontPower = axial + lateral + yawControl;
            double rightFrontPower = axial - lateral - yawControl;
            double leftBackPower = axial - lateral + yawControl;
            double rightBackPower = axial + lateral - yawControl;

            // Calculate and send power to drivetrain
            robot.fl.setPower(leftFrontPower * mainThrottle);
            robot.fr.setPower(rightFrontPower * mainThrottle);
            robot.bl.setPower(leftBackPower * mainThrottle);
            robot.br.setPower(rightBackPower * mainThrottle);

            // Limelight things!
            robot.limelightThread.scanGoalAngle();
            robot.limelightThread.scanObelisk();

            telemetry.addData("Launcher Thread Alive", robot.launcherThread.isAlive());
            telemetry.addData("Launcher Power", robot.idealLauncherPower);
            telemetry.addData("\nTx", robot.goalTx);
            telemetry.addData("Ty", robot.goalTy);

            telemetry.addLine("\n\nControls:\n" +
                    "  Gamepad1:\n" +
                    "    Axial Control - Left Stick Y\n" +
                    "    Lateral Control - Left Stick X\n" +
                    "    Yaw Control - Right Stick X\n" +
                    "    Throttle - Right Trigger\n" +
                    "    FCD Reset - dPad Up\n" +
                    "  Gamepad2:\n" +
                    "    Launch One Artifact - A\n" +
                    "    Launch Three Artifacts - Y\n" +
                    "    Agitator Assembly - B\n" +
                    "    Increase Launcher Power - dPad Up\n" +
                    "    Decrease Launcher Power - dPad Down");

            telemetry.update();
        }
    }
}
