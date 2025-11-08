package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.Config;

@Autonomous(name = "Auto", preselectTeleOp = "SOLO: TeleOp")
public class Auto extends LinearOpMode {
    Config robot;
    int startDelay = 0;
    enum StartingPosition{
        POSITION1,
        POSITION2,
        POSITION2CORRECT;
    }
    Config.Alliance alliance;
    StartingPosition startingPosition = StartingPosition.POSITION2CORRECT;

    @Override
    public void runOpMode() {
        robot = new Config(this);
        robot.init();
        robot.initTweetyBird();

        runSelector();

        waitForStart();

        telemetry.addData("Currently in Start Delay",startDelay);
        telemetry.update();
        sleep(startDelay);

        switch (startingPosition){
            case POSITION1: position1Auto(); break;
            case POSITION2: position2Auto(); break;
            case POSITION2CORRECT: position2AutoCorrect(); break;
        }


    }

    void position1Auto() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(-1,50,0);
        robot.tweetyBird.addWaypoint(-2,84,132);
        robot.tweetyBird.waitWhileBusy();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(-1,50,132);
        robot.tweetyBird.waitWhileBusy();
    }

    void position2Auto() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(1,50,0);
        robot.tweetyBird.addWaypoint(2,84,-120);
        robot.tweetyBird.waitWhileBusy();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(1,50,-120);
        robot.tweetyBird.waitWhileBusy();
    }

    void position2AutoCorrect() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(1,50,0);
        robot.tweetyBird.addWaypoint(2,84,-120);
        robot.tweetyBird.waitWhileBusy();
        robot.limelightThread.startGoalCorrection();
        robot.limelightThread.waitWhileCorrecting();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(1,50,-120);
        robot.tweetyBird.waitWhileBusy();
    }

    private void runSelector() {
        // Read last alliance from file
        robot.alliance = robot.readAllianceFromFile();

        // Selection variables
        String[] allianceOptions = {"Red", "Blue"};
        String[] runAutoOptions = {"YES", "NO"};
        String[] positionOptions = {"One", "Two", "Three"};
        Integer[] delayOptions = {0, 1, 2, 3, 5, 10, 15, 20};

        // Current selections (defaults)
        int allianceIndex = (robot.alliance == Config.Alliance.BLUE) ? 1 : 0;
        int runAutoIndex = 0;
        int positionIndex = 0;
        int delayIndex = 0;

        // Which entry we're on (0-3)
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
                else if (currentEntry == 1) runAutoIndex = (runAutoIndex + 1) % runAutoOptions.length;
                else if (currentEntry == 2) positionIndex = (positionIndex + 1) % positionOptions.length;
                else if (currentEntry == 3) delayIndex = (delayIndex + 1) % delayOptions.length;
            }

            if (dpadDownPressed) {
                // Decrement current entry's value
                if (currentEntry == 0) allianceIndex = (allianceIndex - 1 + allianceOptions.length) % allianceOptions.length;
                else if (currentEntry == 1) runAutoIndex = (runAutoIndex - 1 + runAutoOptions.length) % runAutoOptions.length;
                else if (currentEntry == 2) positionIndex = (positionIndex - 1 + positionOptions.length) % positionOptions.length;
                else if (currentEntry == 3) delayIndex = (delayIndex - 1 + delayOptions.length) % delayOptions.length;
            }

            if (dpadRightPressed) {
                // Move to next entry
                currentEntry = (currentEntry + 1) % 4;
            }

            if (dpadLeftPressed) {
                // Move to previous entry
                currentEntry = (currentEntry - 1 + 4) % 4;
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
            String prefix2 = (currentEntry == 2) ? "> " : "  ";
            String prefix3 = (currentEntry == 3) ? "> " : "  ";

            telemetry.addLine(prefix0 + "Alliance: < " + allianceOptions[allianceIndex] + " >");
            telemetry.addLine(prefix1 + "Run Auto: < " + runAutoOptions[runAutoIndex] + " >");
            telemetry.addLine(prefix2 + "Position: < " + positionOptions[positionIndex] + " >");
            telemetry.addLine(prefix3 + "Delay (sec): < " + delayOptions[delayIndex] + " >");

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

        robot.runAuto = runAutoOptions[runAutoIndex].equals("YES");

        switch (positionOptions[positionIndex]) {
            case "One": robot.startingPosition = 1; break;
            case "Two": robot.startingPosition = 2; break;
            case "Three": robot.startingPosition = 3; break;
        }

        robot.startingDelay = delayOptions[delayIndex];

        // Final confirmation display
        telemetry.addLine("=== CONFIRMED ===");
        telemetry.addLine("Alliance: " + robot.alliance);
        telemetry.addLine("Run Auto: " + robot.runAuto);
        telemetry.addLine("Position: " + robot.startingPosition);
        telemetry.addLine("Delay: " + robot.startingDelay);
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();
    }
}
