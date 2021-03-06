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
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="2 Player Garbagio", group="Iterative Opmode")
//@Disabled
public class MS_2PlayerGarbagio extends OpMode
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

        rArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        rArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lefty = gamepad1.left_stick_y;
        double leftx = -gamepad1.left_stick_x;
        double rightx = -gamepad1.right_stick_x;

        //Use math to figure out how to power motors (CREDIT: dmssargent on FTC Forums)
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x * -1) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * -1;
        final double v1 = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;//Range.clip(r * Math.cos(robotAngle) + rightX, -1, 1);
        final double v2 = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;//Range.clip(r * Math.sin(robotAngle) - rightX, -1, 1);
        final double v3 = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;//Range.clip(r * Math.sin(robotAngle) + rightX, -1, 1);
        final double v4 = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;//Range.clip(r * Math.cos(robotAngle) - rightX, -1, 1);

        // Send calculated power to wheels
        if (gamepad1.a) {
            lfDrive.setPower(v1);
            rfDrive.setPower(v2);
            lbDrive.setPower(v3);
            rbDrive.setPower(v4);
        } else {
            lfDrive.setPower(Range.clip(v1, -0.5, 0.5));
            rfDrive.setPower(Range.clip(v2, -0.5, 0.5));
            lbDrive.setPower(Range.clip(v3, -0.5, 0.5));
            rbDrive.setPower(Range.clip(v4, -0.5, 0.5));
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "lf %s, rf %s, lb %s, rb %s", lfDrive.getCurrentPosition(), rfDrive.getCurrentPosition(), lbDrive.getCurrentPosition(), rbDrive.getCurrentPosition());

        if (gamepad2.a) {
            armServo.setPosition(1);
        } else if (gamepad2.b) {
            armServo.setPosition(0.5);
        } else if (gamepad1.x) {
            rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.right_trigger > 0) {
            rArm.setPower(-0.3);
            lArm.setPower(-0.3);
        } else if (gamepad2.left_trigger > 0) {
            rArm.setPower(0.3);
            lArm.setPower(0.3);
        } else {
            rArm.setPower(0);
            lArm.setPower(0);
        }

        telemetry.addData("Arms: ", "Right: %s | Left: %s", rArm.getCurrentPosition(), lArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
