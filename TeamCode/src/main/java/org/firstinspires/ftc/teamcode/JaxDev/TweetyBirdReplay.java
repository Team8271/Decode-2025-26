package org.firstinspires.ftc.teamcode.JaxDev;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@Autonomous(name = "TweetyBird Replay", group = "poc")
public class TweetyBirdReplay extends LinearOpMode {
  // Pulling configuration
  private final JaxDevConfig robot = new JaxDevConfig(this);

  // Defining variables to be loaded
  private JSONObject recordedMatch;
  private JSONArray recordedActions;

  // Opmode
  @Override
  public void runOpMode() {
    // Initializing robot
    robot.init();
    robot.initTweetyBird();

    // Loading autonomous file
    telemetry.addLine("Loading last replay...");
    telemetry.update();
    try {
      // Fetching file
      File recordFile = new File(Environment.getExternalStorageDirectory(), "tweetyBirdRecord.json");
      // Parsing file
      BufferedReader buffer = new BufferedReader(new FileReader(recordFile));
      StringBuilder builder = new StringBuilder();
      String line;
      while ((line = buffer.readLine()) != null && opModeInInit()) {
        builder.append(line).append("\n");
      }
      // Setting variables
      recordedMatch = new JSONObject(builder.toString());
      recordedActions = recordedMatch.getJSONArray("actions");
    } catch (JSONException | IOException e) {
      throw new RuntimeException(e);
    }
    telemetry.addLine("Loaded " + recordedActions.length() + " keys!");
    telemetry.addLine();

    // Waiting for start
    telemetry.addLine("Robot is ready");
    telemetry.update();
    waitForStart();

    // Parsing actions
    for (int i = 0; i < recordedActions.length(); i++) {
      // Break if opmode ended
      if (!opModeIsActive()) {
        break;
      }

      try {
        // Loading current action
        JSONObject currentAction = recordedActions.getJSONObject(i);

        // Telemetry
        telemetry.addLine("Currently running replay, on key "+
                (i+1) +"/"+ recordedActions.length() + "...");
        telemetry.addLine();
        telemetry.addLine("Current key data:");
        telemetry.addLine(currentAction.toString());
        telemetry.update();

        // Waypoint action
        if (currentAction.get("type").equals("waypoint")) {
          robot.tweetyBird.sendTargetPosition(
                  currentAction.getDouble("x"),
                  currentAction.getDouble("y"),
                  Math.toDegrees(currentAction.getDouble("z"))
          );
        }

        // Wait action
        if (currentAction.get("type").equals("wait")) {
          robot.tweetyBird.waitWhileBusy();
        }
      } catch (JSONException e) {
        throw new RuntimeException(e);
      }
    }

    // Ending opmode
    robot.tweetyBird.waitWhileBusy();
    robot.tweetyBird.close();
  }
}
