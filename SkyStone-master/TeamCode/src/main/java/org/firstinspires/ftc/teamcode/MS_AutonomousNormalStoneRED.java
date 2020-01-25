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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Auto-GENERIC-StoneBoi-RED", group="Linear Opmode")
//@Disabled
public class MS_AutonomousNormalStoneRED extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lbDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;

    private DcMotor rArm = null;
    private DcMotor lArm = null;

    private Servo armServo = null;

    private int[] down_position = {-1600, -1600};
    private int[] back_position = {0, 0};
    private int[] drop_position = {-543, -524};
    private int[] holding_position = {160, 40};
    private int[] holding2_position = {-1003, -999};

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            joystickDrive(0, 1, 0, 0);
            sleep(1000);
            stopDrive();
            pickupStone();
            joystickDrive(0, 0, 1, 0);
            sleep(1250);
            joystickDrive(0, 1, 0, 0);
            sleep(3250);
            stopDrive();
            armServo.setPosition(0.5);
            sleep(100);
            joystickDrive(0,-1, 0, 0);
            sleep(1000);
            stopDrive();
        }
    }

    private void leftSideMotorPower() {
        lfDrive.setPower(.1);
        lbDrive.setPower(.1);

    }

    private void joystickDrive(double leftx, double lefty, double rightx, double righty) {
        lefty *= -1;

        final double v1 = lefty - leftx - rightx;
        final double v2 = lefty + leftx + rightx;
        final double v3 = lefty + leftx - rightx;
        final double v4 = lefty - leftx + rightx;

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send calculated power to wheels
        lfDrive.setPower(Range.clip(v1, -0.25, 0.25));
        rfDrive.setPower(Range.clip(v2, -0.25, 0.25));
        lbDrive.setPower(Range.clip(v3, -0.25, 0.25));
        rbDrive.setPower(Range.clip(v4, -0.25, 0.25));
    }

    private int moveArm(int posIndex, int errorMargin) {
        if (posIndex == 0) {
            rArm.setTargetPosition(down_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(down_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (posIndex == 1) {
            rArm.setTargetPosition(drop_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(drop_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (posIndex == 2) {
            rArm.setTargetPosition(back_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(back_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (posIndex == 3) {
            rArm.setTargetPosition(holding_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(holding_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (posIndex == 4) {
            rArm.setTargetPosition(holding2_position[0]);
            rArm.setPower(0.3);
            lArm.setTargetPosition(holding2_position[1]);
            lArm.setPower(0.3);
            rArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (Math.abs(((lArm.getTargetPosition()) - (lArm.getCurrentPosition()))) > errorMargin) {
            rArm.setPower(0.3);
            lArm.setPower(0.3);
            telemetry.addData("Arms: ", "Right: %s | Left: %s", rArm.getCurrentPosition(), lArm.getCurrentPosition());
            telemetry.update();
        }
        rArm.setPower(0);
        lArm.setPower(0);
        rArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return posIndex;
    }

    private void stopDrive() {
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }

    private void pickupStone() {
        rArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armServo.setPosition(0.5);
        moveArm(0, 5);
        armServo.setPosition(1);
        sleep(1000);
        moveArm(3, 5);
        sleep(500);
    }
}
