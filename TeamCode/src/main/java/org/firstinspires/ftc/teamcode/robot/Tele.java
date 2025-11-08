package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Config;

@TeleOp(name = "TeleOp")
public class Tele extends LinearOpMode {

    Config robot;
    ElapsedTime runtime;
    double lastTime = 0;

    @Override
    public void runOpMode() {
        robot = new Config(this);
        robot.init();
        runtime = new ElapsedTime();
        boolean debounce = false;
        boolean launcherDebounce = false;
        double launcherPower = 0.8;
        double agitatorPower = 0;
        double intakeServoTarget = 0.5;

        runSelector();

        waitForStart();
        runtime.reset();

        robot.kickerMotor.setPower(robot.kickerIdlePower);

        while (opModeIsActive()) {

            double axialControl;
            double lateralControl;
            double yawControl;
            double mainThrottle;
            boolean resetFCD;

            boolean launchOneArtifact;
            boolean launchTwoArtifacts;
            boolean activateAgitatorAssembly;
            boolean activateAimAssist = false;
            boolean abort = false;

            // One Driver
            if(robot.numberOfDrivers == 1) {
                // Merged Driver Controls
                axialControl = -gamepad1.left_stick_y;  // y axis
                lateralControl = gamepad1.left_stick_x; // x axis
                yawControl = gamepad1.right_stick_x;    // z axis
                mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
                resetFCD = gamepad1.dpad_up; // z axis reset

                launchOneArtifact = gamepad1.a;
                launchTwoArtifacts = gamepad1.y;
                activateAgitatorAssembly = gamepad1.b;
                activateAimAssist = gamepad1.dpad_right;
                abort = gamepad1.options;

            }
            // Two Drivers
            else {
                // Driver 1 Controls
                axialControl = -gamepad1.left_stick_y;  // y axis
                lateralControl = gamepad1.left_stick_x; // x axis
                yawControl = gamepad1.right_stick_x;    // z axis
                mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
                resetFCD = gamepad1.dpad_up; // z axis reset

                // Driver 2 Controls
                launchOneArtifact = gamepad2.a;
                launchTwoArtifacts = gamepad2.y;
                activateAgitatorAssembly = gamepad2.b;
            }


            if(activateAimAssist) {
                robot.limelightThread.startGoalCorrection();
                while(!robot.limelightThread.isGoalCorrectionDone() && !isStopRequested()){
                    if(abort){
                        robot.limelightThread.terminateGoalCorrection();
                        break;
                    }
                    telemetry.addLine("Looping aim assist");
                    telemetry.update();
                }
            }

            if (launchOneArtifact) {
                robot.launcherThread.launch(1);
            }
            if (launchTwoArtifacts) {
                robot.launcherThread.launch(3);
            }

            if(gamepad1.right_bumper && !launcherDebounce) {
                launcherPower += 0.1;
                launcherDebounce = true;
            }
            if(gamepad1.left_bumper && !launcherDebounce) {
                launcherPower -= 0.1;
                launcherDebounce = true;
            }

            if(!gamepad1.right_bumper && !gamepad1.left_bumper && launcherDebounce){
                launcherDebounce = false;
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

            robot.limelightThread.scanGoalAngle();


            robot.idealLauncherPower = launcherPower;

            telemetry.addData("Launcher Thread Alive", robot.launcherThread.isAlive());
            telemetry.addData("Launcher Power", robot.idealLauncherPower);
            telemetry.addData("\nTx", robot.limelightThread.getGoalTx());
            telemetry.addData("Ty", robot.limelightThread.getGoalTy());

            telemetry.addData("X",robot.odometer.getX());
            telemetry.addData("Y",robot.odometer.getY());
            telemetry.addData("Z",Math.toDegrees(robot.odometer.getZ()));

            telemetry.addLine("\n\nControls:");
            // One Driver
            if(robot.numberOfDrivers == 1){
                telemetry.addLine("" +
                        "  Axial Control - Left Stick Y\n" +
                        "  Lateral Control - Left Stick X\n" +
                        "  Yaw Control - Right Stick X\n" +
                        "  Throttle - Right Trigger\n" +
                        "  FCD Reset - dPad Up\n" +
                        "  Launch One Artifact - A\n" +
                        "  Launch Three Artifacts - Y\n" +
                        "  Agitator Assembly - B");
            }
            // Two Drivers
            else {
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
            }


            telemetry.update();
        }
        robot.killThreads();
    }

    private void runSelector() {
        // Read last alliance from file
        robot.alliance = robot.readAllianceFromFile();

        // Selection variables
        String[] allianceOptions = {"Red", "Blue"};
        Integer[] driverOptions = {2, 1};

        // Current selections (defaults)
        int allianceIndex = (robot.alliance == Config.Alliance.BLUE) ? 1 : 0;
        int driverIndex = 0;

        // Which entry we're on (0-1)
        int currentEntry = 0;

        // Button state tracking for edge detection
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastA = false;

        // Selection loop
        boolean confirmed = false;
        while (!isStarted() && !isStopRequested() && !confirmed) {
            // Get current button states (check both gamepads)
            boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
            boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
            boolean aButton = gamepad1.a || gamepad2.a;

            // Edge detection (only trigger on button press, not hold)
            boolean dpadUpPressed = dpadUp && !lastDpadUp;
            boolean dpadDownPressed = dpadDown && !lastDpadDown;
            boolean dpadLeftPressed = dpadLeft && !lastDpadLeft;
            boolean dpadRightPressed = dpadRight && !lastDpadRight;
            boolean aPressed = aButton && !lastA;

            // Handle input
            if (dpadUpPressed) {
                // Increment current entry's value
                if (currentEntry == 0) allianceIndex = (allianceIndex + 1) % allianceOptions.length;
                else if (currentEntry == 1) driverIndex = (driverIndex + 1) % driverOptions.length;
            }

            if (dpadDownPressed) {
                // Decrement current entry's value
                if (currentEntry == 0) allianceIndex = (allianceIndex - 1 + allianceOptions.length) % allianceOptions.length;
                else if (currentEntry == 1) driverIndex = (driverIndex - 1 + driverOptions.length) % driverOptions.length;
            }

            if (dpadRightPressed) {
                // Move to next entry
                currentEntry = (currentEntry + 1) % 2;
            }

            if (dpadLeftPressed) {
                // Move to previous entry
                currentEntry = (currentEntry - 1 + 2) % 2;
            }

            if (aPressed) {
                // Confirm selection
                confirmed = true;
            }

            // Update last button states
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastA = aButton;

            // Display current selections
            telemetry.addLine("=== CONFIG SELECTOR ===");
            telemetry.addLine();

            String prefix0 = (currentEntry == 0) ? "> " : "  ";
            String prefix1 = (currentEntry == 1) ? "> " : "  ";

            telemetry.addLine(prefix0 + "Alliance: < " + allianceOptions[allianceIndex] + " >");
            telemetry.addLine(prefix1 + "Drivers: < " + driverOptions[driverIndex] + " >");

            telemetry.addLine();
            telemetry.addLine("Controls: DPad Up/Down: Change value");
            telemetry.addLine("          DPad Left/Right: Change entry");
            telemetry.addLine("          A: Confirm");
            telemetry.update();

            sleep(50); // Small delay to prevent excessive looping
        }

        // Apply selections to robot config
        robot.alliance = allianceOptions[allianceIndex].equals("Blue") ?
                Config.Alliance.BLUE : Config.Alliance.RED;
        robot.saveAllianceToFile(robot.alliance);

        robot.numberOfDrivers = driverOptions[driverIndex];

        // Final confirmation display
        telemetry.addLine("=== CONFIRMED ===");
        telemetry.addLine("Alliance: " + robot.alliance);
        telemetry.addLine("Drivers: " + robot.numberOfDrivers);
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();
    }

}


