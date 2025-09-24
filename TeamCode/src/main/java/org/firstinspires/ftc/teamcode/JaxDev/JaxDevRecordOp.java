package org.firstinspires.ftc.teamcode.JaxDev;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Entry class
 */
@TeleOp(name = "Jax Record")
public class JaxDevRecordOp extends LinearOpMode {
  // Loading robot configuration
  private final JaxDevConfig robot = new JaxDevConfig(this);

  // Creating json array
  public JSONObject mainRecord= new JSONObject();
  public JSONArray recordArray = new JSONArray();

  /**
   * Runtime
   */
  @Override
  public void runOpMode() {
    // Starting robot
    robot.init();

    // Waiting for drivers to start
    telemetry.addLine("Robot is ready");
    telemetry.update();
    waitForStart();

    // Debounce values
    boolean recordDebounce = false;

    // Runtime loop
    while (opModeIsActive()) {
      // Controls for gamepad 1
      double axialControl = -gamepad1.left_stick_y;
      double lateralControl = gamepad1.left_stick_x;
      double yawControl = gamepad1.right_stick_x;
      double throttleControl = (gamepad1.right_trigger/0.8)+0.2;
      boolean fcdReset = gamepad1.left_bumper;
      boolean record = gamepad1.a;
      boolean recordStop = gamepad1.b;

      // Controls for gamepad 2
      double axialPullControl = -gamepad2.left_stick_y;
      double lateralPullControl = -gamepad2.left_stick_x;

      // Field centric yaw reset
      if (fcdReset) {
        robot.odometer.resetTo(0,0,0);
      }

      // Field centric calculations
      double gamepadRad = Math.atan2(lateralControl, axialControl);
      double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
      double robotRadians = robot.odometer.getZ();
      double targetRad = gamepadRad - robotRadians;

      // Pull via controller
      double pullRad = Math.atan2(lateralPullControl, axialPullControl);
      double pullStrength = Range.clip(Math.hypot(lateralPullControl, axialPullControl), 0, 1);

      // Finalized calculations
      double finalRad = targetRad-(((Math.abs(targetRad-pullRad))*(targetRad/Math.abs(targetRad)))*pullStrength);
      double axial = Math.cos(finalRad)*gamepadHypot;
      double lateral = Math.sin(finalRad)*gamepadHypot;

      // Setting motor powers
      setMovementPower(axial, lateral, yawControl, throttleControl);

      // Recording
      if (record && !recordDebounce) {
        recordDebounce = true;
          try {
              recordPoint();
          } catch (JSONException e) {
              throw new RuntimeException(e);
          }
      }

      if (recordStop && !recordDebounce) {
        recordDebounce = true;
          try {
              recordStop();
          } catch (JSONException e) {
              throw new RuntimeException(e);
          }
      }

      if (!record && !recordStop) {
        recordDebounce = false;
      }

      // Telemetry
      telemetry.addData("X",robot.odometer.getX());
      telemetry.addData("Y",robot.odometer.getY());
      telemetry.addData("Z",robot.odometer.getZ());
      telemetry.update();
    }

    // Writing recording
    try {
      mainRecord.put("actions",recordArray);
      File recordFile = new File(Environment.getExternalStorageDirectory(), "tweetyBirdRecord.json");
      FileWriter recordWriter = new FileWriter(recordFile);
      recordWriter.write(mainRecord.toString(2));
      recordWriter.flush();
      recordWriter.close();
    } catch (IOException | JSONException e) {
      throw new RuntimeException(e);
    }

  }

  public void recordPoint() throws JSONException {
    JSONObject newRecord = new JSONObject();
    newRecord.put("type","waypoint");
    newRecord.put("x",robot.odometer.getX());
    newRecord.put("y",robot.odometer.getY());
    newRecord.put("z",robot.odometer.getZ());
    recordArray.put(newRecord);
  }

  public void recordStop() throws JSONException {
    recordPoint();
    JSONObject newRecord = new JSONObject();
    newRecord.put("type","wait");
    recordArray.put(newRecord);
  }

  public void setMovementPower(double axial, double lateral, double yaw, double speed) {
    setBreaking(axial == 0 && lateral == 0 && yaw == 0);

    double frontLeftPower  = ((axial + lateral + yaw) * speed);
    double frontRightPower = ((axial - lateral - yaw) * speed);
    double backLeftPower   = ((axial - lateral + yaw) * speed);
    double backRightPower  = ((axial + lateral - yaw) * speed);
    robot.fl.setPower(frontLeftPower);
    robot.fr.setPower(frontRightPower);
    robot.bl.setPower(backLeftPower);
    robot.br.setPower(backRightPower);
  }

  public void setBreaking(boolean choice) {
    if (choice) {
      robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } else {
      robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
  }
}
