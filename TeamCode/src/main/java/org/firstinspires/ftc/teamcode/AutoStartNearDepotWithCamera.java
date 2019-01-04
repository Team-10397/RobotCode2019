/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto Start Near Depot Camera", group = "Camera")
//@Disabled
public class AutoStartNearDepotWithCamera extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
    private static final String VUFORIA_KEY = "AdCuaEX/////AAABmXYJgRHZxkB9gj+81cIaX+JZm4W2w3Ee2HhKucJINnuXQ8l214BoCiyEk04zmQ/1VPvo+8PY7Um3eI5rI4WnSJmEXo7jyMz2WZDkkRnA88uBCtbml8CsMSIS7J3aUcgVd9P8ocLLgwqpavhEEaUixEx/16rgzIEtuHcq5ghQzzCkqR1xvAaxnx5lWM+ixf6hBCfZEnaiUM7WjD4gflO55IpoO/CdCWQrGUw2LuUKW2J+4K6ftKwJ+B1Qdy7pt2tDrGZvMyB4AcphPuoJRCSr5NgRoNWZ+WH5LqAdzYEO0Bv7C9LeSgmSPPT7GPPDpjv6+3DO5BE6l+2uMYQQbuF11BWKKq5Xp+D5Y6l2+W97zpgP";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    HardwareRobot iceRobot = new HardwareRobot();
    int state = 0; // TODO: use enums
    int goldSpot = 0;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        iceRobot.init(hardwareMap);




        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(); // initializes the camera
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate(); // initializes the mineral detection
        }


        waitForStart();
        this.resetStartTime(); // resets timer (for debugging purposes)

        if (opModeIsActive()) {
            while (goldSpot == 0 && opModeIsActive()) { //this loop runs until an object is detected or the program stops
                telemetry.addData("scanning", this.getRuntime()); //updates the display with the time since the scan started
                telemetry.update();
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // puts all of the scanned minerals in a list
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 2) { // the following code runs if the robot sees 2 minerals
                            int goldMineralX = -1; //  defining minerals
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) { // code loops for every mineral that is detected
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getTop(); // if a gold mineral is detected, its position is recorded
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getTop(); // if silver elements are detected, their positions are recorded.
                                } else {
                                    silverMineral2X = (int) recognition.getTop();
                                }
                            }
                            if (silverMineral1X != -1 && silverMineral2X != -1 && goldMineralX == -1) { //if two silver minerals are found, then the gold is on the left [] O O (the robot only scans for the right two minerals)
                                goldSpot = 1; //left
                            } else if (goldMineralX < silverMineral1X) { // gold to the left of silver: O [] O
                                goldSpot = 2; //center
                            } else if (goldMineralX > silverMineral1X) { // gold to the right of silver: O O []
                                goldSpot = 3; //right
                            }
                        }
                    }
                }
            }
        }
        telemetry.addData("gold is",goldSpot); //  displays the position of the gold mineral
        telemetry.addData("time took", this.getRuntime()+" seconds"); // displays the time taken to scan
        telemetry.update();
        while (opModeIsActive()){
            telemetry.update();
            switch (state){
                case 0: // drops from the lander
                    iceRobot.climbMotor.setPower(-1);
                    if (iceRobot.climbMotor.getCurrentPosition() < -3000){
                        iceRobot.climbMotor.setPower(0);
                        state += 1;
                    }
                    break;
                case 1: // turns to unlatch from lander
                    iceRobot.encoderTurn(15);
                    sleep(500);
                    iceRobot.encoderMove(-3,.50, this);
                    sleep(500);
                    state += 1;
                    break;

                case 2: // lowers climbing arm
                    iceRobot.climbMotor.setPower(1);
                    double start_time = runtime.milliseconds();
                    if (iceRobot.climbMotor.getCurrentPosition() > -1000 || runtime.milliseconds() - start_time > 750){
                        iceRobot.climbMotor.setPower(0);
                        sleep(1000);
                        state += 2;
                    }
                    break;
                case 3: // turns to right itself
                    iceRobot.encoderTurn(-15);
                    sleep(500);
                    state += 1;
                    break;
                case 4: // decides which way to go
                    state += goldSpot;
                    break;
                case 5:
                    iceRobot.encoderTurn(45);
                    state += 1;
                    break;
                case 6:
                    state += 1;
                    break;
                case 7:
                    iceRobot.encoderTurn(-45);
                    state += 1;
                    break;
                case 8: // backs into the depot
                    iceRobot.encoderMove(-50,1, this);
                    sleep(500);
                    state += 1;
                    break;
                case 9: // drops the team marker
                    iceRobot.rightClaw.setPosition(iceRobot.SERVO_CENTER);
                    sleep(1000);
                    state += 2;
                    break;
                case 10: // goes forward to ensure team marker iss dropped
                    iceRobot.encoderTurn(45);
                    sleep(500);
                    iceRobot.stop();
                    state += 1;
                    break;
                case 11:
                    iceRobot.encoderMove(50,.75, this);
                    sleep(500);
                    iceRobot.encoderMove(25,.1, this);
                    state += 1;
                    break;
            }


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
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
