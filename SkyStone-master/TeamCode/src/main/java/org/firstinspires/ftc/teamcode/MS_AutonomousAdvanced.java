/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Autonomous Advanced", group = "Linear OpMode")
//@Disabled
public class MS_AutonomousAdvanced extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

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
    private int[] holding_position = {-1090, -1022};

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYJrLpn/////AAABmZXhHDf3/UFchi0gZqHEb/dbguRcFBu7sG0txh6xZhexnNe83rkWYaR8QREDYR6VEVHlWwHYBBdKqy1cWzgEG6sDxnsYzzGJk664NDPTBjFOCZZKW+RdZOInqPg0KvPp2mjdQovPGPdyHMGuFK08P5d3vOGMXFcxNDNeWw58c2HX9z9xl7cmgztNFIz4wqa94tkpw+BmuUNGO2+pWhXZhyJ55Qdj4LilYQH3nAkBkvFFyJX/uUoG895t4c5H6rUYrTRMQUqoXGQKubJGp4HAXivDG2dRHJHzmsaE91woEOaoiUdta+ynAN8TZMpFuPfg2Mh933qPF/uY3KDt4bNyFQUA/SwPCucARsryYZXZnYu9";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        lbDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rbDrive = hardwareMap.get(DcMotor.class, "backRight");
        lfDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rfDrive = hardwareMap.get(DcMotor.class, "frontRight");

        rArm = hardwareMap.get(DcMotor.class, "rightArm");
        lArm = hardwareMap.get(DcMotor.class, "leftArm");

        armServo = hardwareMap.get(Servo.class, "armServo");

        rbDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);

        rArm.setDirection(DcMotor.Direction.REVERSE);
        //lArm.setDirection(DcMotor.Direction.REVERSE);

        rArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Make sure to put arm in back position before hitting Start.");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        rArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            boolean notInRange = true;
            while (notInRange && opModeIsActive()) {
                Recognition recognition = recognizeStone();
                sleep(500);
                float size = (recognition.getWidth()*recognition.getHeight());
                telemetry.addData("Size: ", size);
                if (size < 49000) { //max is 60000
                    telemetry.addLine("Not in range! " + size);
                    telemetry.update();
                    joystickDrive(0, -1, 0, 0);
                    sleep(500);
                    stopDrive();
                } else if (size > 60000) {
                    joystickDrive(0, 1, 0, 0);
                    sleep(500);
                    stopDrive();
                } else {
                    telemetry.addLine("In range! " + size);
                    telemetry.update();
                    notInRange = false;
                }
            }

            for (int i = 0; i < 6; i++) {
                if (!opModeIsActive()) {
                    break;
                }
                Recognition recognition = recognizeStone();
                if (recognition.getLabel() == "Skystone") {
                    while (recognizeStone().getLeft() > 350) {
                        joystickDrive(0, 0, -0.5, 0);
                        sleep(250);
                        stopDrive();
                        sleep(250);
                    }
                    pickupStone(0);
                    stop();
                    while (opModeIsActive()) {
                        recognition = recognizeStone();
                        telemetry.addData("Dist from left=%s", recognition.getLeft());
                        telemetry.update();
                    }
                    break;
                } else {
                    joystickDrive(-1, 0, 0, 0);
                    sleep(500);
                    stopDrive();
                }
            }

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void pickupStone(int inchesFromDrop) {
        armServo.setPosition(0.5);
        moveArm(0, 25);
        telemetry.addLine("Finished moving!");
        telemetry.update();
        armServo.setPosition(1);
        sleep(1000);
        moveArm(3, 5);
        sleep(500);
        armServo.setPosition(0.5);
        sleep(2000);
//        rbDrive.setPower(-0.5);
//        rfDrive.setPower(-0.5);
//        lbDrive.setPower(0.5);
//        lfDrive.setPower(0.5);
//        sleep(1500);
//        rbDrive.setPower(0);
//        rfDrive.setPower(0);
//        lbDrive.setPower(0);
//        lfDrive.setPower(0);
//        sleep(2000);
//        rbDrive.setPower(-0.5);
//        rfDrive.setPower(-0.5);
//        lbDrive.setPower(-0.5);
//        lfDrive.setPower(-0.5);
//        sleep(2500);
//        rbDrive.setPower(0);
//        rfDrive.setPower(0);
//        lbDrive.setPower(0);
//        lfDrive.setPower(0);
        //strafe in direction of drop
    }

    private void dropStone(int inchesToDrop) {
        moveArm(1, 25);
        armServo.setPosition(0.5);
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
        }
        while (Math.abs(((rArm.getTargetPosition() * -1) - (rArm.getCurrentPosition() * -1))) > errorMargin) {
            rArm.setPower(0.3);
            lArm.setPower(0.3);
        }
        rArm.setPower(0);
        lArm.setPower(0);
        rArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return posIndex;
    }

    private void joystickDrive(double leftStick_x, double leftStick_y, double rightStick_x, double rightStick_y) {
        //Use math to figure out how to power motors (CREDIT: dmssargent on FTC Forums)
        double r = Math.hypot(leftStick_x, leftStick_y);
        double robotAngle = Math.atan2(leftStick_y, leftStick_x * -1) - Math.PI / 4;
        double rightX = rightStick_x * -1;
        final double v1 = Range.clip(r * Math.cos(robotAngle) + rightX, -1, 1);
        final double v2 = Range.clip(r * Math.sin(robotAngle) - rightX, -1, 1);
        final double v3 = Range.clip(r * Math.sin(robotAngle) + rightX, -1, 1);
        final double v4 = Range.clip(r * Math.cos(robotAngle) - rightX, -1, 1);

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send calculated power to wheels
        lfDrive.setPower(v1);
        rfDrive.setPower(v2);
        lbDrive.setPower(v3);
        rbDrive.setPower(v4);

        telemetry.addLine(String.valueOf(v1));
        telemetry.addLine(String.valueOf(v2));
        telemetry.addLine(String.valueOf(v3));
        telemetry.addLine(String.valueOf(v4));
    }

    private void stopDrive() {
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minimumConfidence = 0.8;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private Recognition recognizeStone() {
        boolean recognizing = true;
        while (recognizing && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        return recognition;
                    }
                    telemetry.update();
                }
            }
        }
        return null;
    }
}
