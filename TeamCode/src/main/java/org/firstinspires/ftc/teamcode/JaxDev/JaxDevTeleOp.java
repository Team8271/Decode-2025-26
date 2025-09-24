package org.firstinspires.ftc.teamcode.JaxDev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Jax Development TeleOp")
public class JaxDevTeleOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        JaxDevConfig robot = new JaxDevConfig(this);
        robot.init();
        boolean debounce = true;

        while(opModeInInit() && !isStopRequested()){
            telemetry.addLine("Initialized");
            telemetry.addLine("\nPress 'D-Pad Down' to change team.");
            telemetry.addData("Team",robot.team);

            if(gamepad1.dpad_down || gamepad2.dpad_down && debounce){
                debounce = false;
                switch(robot.team){
                    case RED:
                        robot.setTeam(JaxDevConfig.Team.BLUE);
                        break;
                    case BLUE:
                        robot.setTeam(JaxDevConfig.Team.RED);
                        break;
                }
            }
            if(!gamepad1.dpad_down && !gamepad2.dpad_down && !debounce){
                debounce = true;
            }

            telemetry.update();
        }

        waitForStart();


        while(opModeIsActive()){

            /// Driver One Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset

            // FCD reset
            if(resetFCD){
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

            // Output robot odometry to telemetry
            String pos = Math.round(robot.odometer.getX()) + ", " + Math.round(robot.odometer.getY()) + ", " + Math.round(Math.toDegrees(robot.odometer.getZ()));
            telemetry.addData("Odometry",pos);

            robot.scanObelisk();
            telemetry.addData("Motif",robot.motif);

            robot.scanGoalAngle();
            String goalAngle = "Goal Tx:" + Math.round(robot.goalTx*100) + " Ty:" + Math.round(robot.goalTy*100);
            telemetry.addLine(goalAngle);

            telemetry.update();

        }
    }
}
