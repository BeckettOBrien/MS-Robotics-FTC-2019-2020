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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

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
@TeleOp(name = "LeftAuto-Test", group = "Concept")
//@Disabled
public class MS_LeftAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final double DIST_FROM_NEAREST_STONE = 55.0; // MILLIMETERS -- Change depending on distance from current position to arm pickup
    private static final double SPEED_DIST_MULTIPLYER = 2.0; // MILLISECONDS -- Number of milliseconds to travel 1 millimeter

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
    private int[] holding_position = {-996, -1050};

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

        Recognition rightmostStone = null;
        Recognition skystone = null;

        boolean finishedDetection = false;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !finishedDetection) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions == null) {
                        continue;
                    }

                    for (int i = 0; i < updatedRecognitions.size(); i++) {
                        if (rightmostStone != null) {
                            if (updatedRecognitions.get(i).getRight() > rightmostStone.getRight()) {
                                rightmostStone = updatedRecognitions.get(i);
                            }
                        } else {
                            rightmostStone = updatedRecognitions.get(i);
                        }
                        if (skystone != null) {
                            if (updatedRecognitions.get(i).getLabel().equals("Skystone")) {
                                skystone = updatedRecognitions.get(i);
                            }
                        } else {
                            skystone = updatedRecognitions.get(i);
                        }
                        if (skystone != null && rightmostStone != null) {
                            finishedDetection = true;
                            break;
                        }
                    }
                }
            }

            // Get joystick value here
            TranslationInfo stoneDirections = getTranslationInfo(skystone, rightmostStone);
            // Start moving with joystick values
            joystickDrive(stoneDirections.joystickCoords[0], stoneDirections.joystickCoords[1], 0, 0);
            sleep((long)(stoneDirections.distFromStone * SPEED_DIST_MULTIPLYER));
            stopDrive();
            pickupStone();
            sleep(100);
            joystickDrive(0, 0, 1, 0);
            sleep(1500);
            joystickDrive(0, 1, 0, 0);
            sleep(5000);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
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
       tfodParameters.minimumConfidence = 0.6;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private TranslationInfo getTranslationInfo(Recognition skystone, Recognition rightmostStone) {
        double distFromLeft = skystone.getLeft(); // recognition.getLeft() in MILLIMETERS

        double distFromStone = Math.sqrt(Math.pow(Math.abs(rightmostStone.getRight() - distFromLeft), 2) + Math.pow(DIST_FROM_NEAREST_STONE, 2)); // Pythagorean Theorem
        double angleFacingStone = Math.toRadians((Math.atan(distFromLeft / DIST_FROM_NEAREST_STONE)) + 90); // Hypotenuse/Adjacent

        double[] joystickCoords = {Math.cos(angleFacingStone), Math.sin(angleFacingStone)}; // Find point on circle from angle

        telemetry.addData("Distance From Left:", distFromLeft);
        telemetry.addData("Hypotenuse:", distFromStone);
        telemetry.addData("Angle:", (Math.atan(distFromLeft / DIST_FROM_NEAREST_STONE)) + 90);
        telemetry.addLine(String.format("Coords: %s, %s", joystickCoords[0], joystickCoords[1]));
        telemetry.update();

        return new TranslationInfo(distFromLeft, distFromStone, joystickCoords); // Wrap output in custom class TranslationInfo
    }

    private class TranslationInfo {
        double distFromLeft;
        double distFromStone;
        double[] joystickCoords;

        TranslationInfo(double distFromLeft, double distFromStone, double[] joystickCoords) {
            this.distFromLeft = distFromLeft;
            this.distFromStone = distFromStone;
            this.joystickCoords = joystickCoords;
        }
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
        lfDrive.setPower(v1);
        rfDrive.setPower(v2);
        lbDrive.setPower(v3);
        rbDrive.setPower(v4);
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
        moveArm(0, 25);
        armServo.setPosition(1);
        sleep(1000);
        moveArm(3, 5);
        sleep(500);
    }
}
