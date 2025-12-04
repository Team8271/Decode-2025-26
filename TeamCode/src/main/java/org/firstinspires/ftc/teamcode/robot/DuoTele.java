package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Duo Tele")
public class DuoTele extends LinearOpMode {

    Config robot;

    @Override
    public void runOpMode() {
        robot = new Config(this,null);
        robot.init();
        boolean debounce = false;
        boolean launcherDebounce = false;
        boolean launcherVelocityDebounce = false;
        boolean changeAllianceDebounce = false;
        double agitatorPower = 0;
        boolean intakeAssemblyIsActive = false;
        boolean intakeAssemblyIsReversed = false;

        telemetry.addLine("Initialized");
        telemetry.update();

        runSelector();

        waitForStart();

        while (opModeIsActive()) {

            // Split Driver Controls
            // Driver 1 Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset

            // Driver 2 Controls
            boolean launchOneArtifact = gamepad2.a;
            boolean launchThreeArtifacts = gamepad2.y;
            boolean intakeAssemblyToggle = gamepad2.b;
            boolean reverseIntakeAssemblyToggle = gamepad2.x;

            boolean changeAlliance = gamepad2.options;

            if(changeAlliance && !changeAllianceDebounce) {
                switch(robot.alliance) {
                    case RED:
                        robot.setAlliance(Config.Alliance.BLUE);
                        break;
                    case BLUE:
                        robot.setAlliance(Config.Alliance.RED);
                        break;
                }
                gamepad2.rumble(200);
                changeAllianceDebounce = true;
            }
            if(!changeAlliance && changeAllianceDebounce) {
                changeAllianceDebounce = false;
            }

            /*

            if(increaseLauncherVelocity && !launcherVelocityDebounce) {
                robot.idealLauncherVelocity += 100;
                robot.log("Tele - Trying to increase launcher vel");
                launcherVelocityDebounce = true;
            }
            if(decreaseLauncherVelocity && !launcherVelocityDebounce) {
                robot.idealLauncherVelocity -= 100;
                robot.log("Tele - Trying to decrease launcher vel");
                launcherVelocityDebounce = true;
            }
            if (!increaseLauncherVelocity && !decreaseLauncherVelocity && launcherVelocityDebounce) {
                launcherVelocityDebounce = false;
            }

             */


            if (launchOneArtifact && !launcherDebounce) {
                robot.launcherThread.launch(1);
                gamepad1.rumble(0.8,0.8,125);
                gamepad2.rumble(0.8,0.8,125);
                launcherDebounce = true;
            }
            if (launchThreeArtifacts && !launcherDebounce) {
                robot.launcherThread.launch(3);
                gamepad1.rumble(0.8,0.8,125);
                gamepad2.rumble(0.8,0.8,125);
                launcherDebounce = true;
            }
            if (!launchOneArtifact && !launchThreeArtifacts && launcherDebounce) {
                launcherDebounce = false;
            }

            if (intakeAssemblyToggle && !debounce) {
                if(!intakeAssemblyIsActive || intakeAssemblyIsReversed) {
                    robot.runIntakeAssembly();
                    intakeAssemblyIsActive = true;
                }
                else {
                    robot.stopIntakeAssembly();
                    intakeAssemblyIsActive = false;
                }

                intakeAssemblyIsReversed = false;
                debounce = true;
            }
            if (reverseIntakeAssemblyToggle && !debounce) {
                robot.reverseIntakeAssembly();
                intakeAssemblyIsReversed = true;

            }

            if (!intakeAssemblyToggle && !reverseIntakeAssemblyToggle && debounce) {
                debounce = false;
            }

            // FCD reset
            if (resetFCD) {
                robot.odometer.resetTo(0,0,0);
                gamepad1.rumble(100);
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


            //robot.limelightThread.scanGoalAngle();
            //robot.aimAssist.runPowerCalculation();

            telemetry.addData("Launcher Velocity", robot.idealLauncherVelocity);
            telemetry.addData("Intake Velocity", robot.intakeMotor.getVelocity());
            //telemetry.addData("\nTx", robot.goalTx);
            //telemetry.addData("AvgDist", robot.goalAvgDist);

            // One Driver Telemetry
            telemetry.addLine(""+
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

    private void runSelector() {

        // Read last alliance from file
        robot.alliance = robot.readAllianceFromFile();
        robot.driverAmount = robot.readDriverAmountFromFile();

        // Button state tracking for edge detection
        boolean lastDpadDown = false;

        while (!isStarted() && !isStopRequested()) {

            boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean dpadDownPressed = dpadDown && !lastDpadDown;

            if (dpadDownPressed) {

                switch (robot.alliance) {
                    case BLUE:
                        robot.setAlliance(Config.Alliance.RED);
                        break;
                    case RED:
                        robot.setAlliance(Config.Alliance.BLUE);
                        break;
                }

            }

            lastDpadDown = dpadDown;

            telemetry.addData("Alliance", robot.alliance.toString());
            telemetry.addLine("\nPress START");
            telemetry.update();

            // Don't hog system
            sleep(50);
        }
        robot.saveAllianceToFile(robot.alliance);
    }
}