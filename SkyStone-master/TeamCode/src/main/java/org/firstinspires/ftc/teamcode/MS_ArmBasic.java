/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**
 Far back:
 Right-0 Left-0

 Far Forwards:
 Right-(-)1191 Left-(-)1166

 Drop Position:
 Right-(-)543 Left-(-)524
 */

@TeleOp(name="Arm Basic", group="Iterative Opmode")
//@Disabled
public class MS_ArmBasic extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lbDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;

    private DcMotor rArm = null;
    private DcMotor lArm = null;

    private Servo armServo = null;

    private int[] down_position = {-1191, -1166};
    private int[] back_position = {0, 0};
    private int[] drop_position = {-543, -524};

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lbDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rbDrive = hardwareMap.get(DcMotor.class, "backRight");
        lfDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rfDrive = hardwareMap.get(DcMotor.class, "frontRight");

        rArm = hardwareMap.get(DcMotor.class, "rightArm");
        lArm = hardwareMap.get(DcMotor.class, "leftArm");

        armServo = hardwareMap.get(Servo.class, "armServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rbDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);

        rArm.setDirection(DcMotor.Direction.REVERSE);
        //lArm.setDirection(DcMotor.Direction.REVERSE);

        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Make sure to put arm in back position before hitting Start.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //armServo.setPosition(0);
        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Use math to figure out how to power motors (CREDIT: dmssargent on FTC Forums)
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * -1;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        // Send calculated power to wheels
        lfDrive.setPower(v1);
        rfDrive.setPower(v2);
        lbDrive.setPower(v3);
        rbDrive.setPower(v4);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        if (gamepad1.a) {
            armServo.setPosition(0);
        } else if (gamepad1.b) {
            armServo.setPosition(0.5);
        }

        if ((rArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION) && (lArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION)) {
            /*if ((rArm.getTargetPosition() == down_position[0]) && (rArm.getCurrentPosition() > down_position[0])) {
                rArm.setPower(0);
                rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if ((lArm.getTargetPosition() == down_position[1]) && (lArm.getCurrentPosition() > down_position[1])) {
                lArm.setPower(0);
                lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }*/

            if ((rArm.getCurrentPosition() == down_position[0]) || (rArm.getCurrentPosition() == back_position[0]) || (rArm.getCurrentPosition() == drop_position[0])) {
                rArm.setPower(0);
                lArm.setPower(0);
                rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if ((lArm.getCurrentPosition() == down_position[1]) || (lArm.getCurrentPosition() == back_position[1]) || (lArm.getCurrentPosition() == drop_position[1])) {
                rArm.setPower(0);
                lArm.setPower(0);
                rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (Math.abs(rArm.getTargetPosition() - rArm.getCurrentPosition()) < 30) {
                //rArm.setPower(0.1);
            }
            if (Math.abs(lArm.getTargetPosition() - lArm.getCurrentPosition()) < 30) {
                //lArm.setPower(0.1);
            }
        }

        if (gamepad1.right_trigger > 0) {
            rArm.setTargetPosition(down_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(down_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.left_trigger > 0) {
            rArm.setTargetPosition(drop_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(drop_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.x) {
            rArm.setTargetPosition(back_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(back_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

/*
        if (gamepad1.left_bumper) {
            rArm.setPower(0.3);
            lArm.setPower(0.3);
        } else if (gamepad1.right_bumper) {
            rArm.setPower(-0.3);
            lArm.setPower(-0.3);
        } else {
            rArm.setPower(0);
            lArm.setPower(0);
        }
 */

        telemetry.addData("Arms", "Right: " + rArm.getCurrentPosition() + " Left: " + lArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
