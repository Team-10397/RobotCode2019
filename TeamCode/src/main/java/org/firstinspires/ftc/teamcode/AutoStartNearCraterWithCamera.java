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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.List;


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

@Autonomous(name="Autonomous Crater With Camera", group="Camera")
public class AutoStartNearCraterWithCamera extends LinearOpMode {

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
        while (opModeIsActive()){
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
                case 4: // backs towards the crater
                    iceRobot.encoderMove(-25,1, this);
                    sleep(500);
                    iceRobot.encoderMove(10,.5,this);
                    sleep(500);
                    state += 1;
                    break;
                case 5:
                    iceRobot.encoderTurn(90);
                    sleep(500);
                    iceRobot.encoderMove(40,.8,this);
                    sleep(500);
                    iceRobot.encoderTurn(-10);
                    sleep(500);
                    iceRobot.encoderMove(40,1,this);
                    sleep(500);
                    iceRobot.encoderTurn(-180);
                    sleep(500);
                    state += 1;
                    break;
                case 6: // drops the team marker
                    iceRobot.rightClaw.setPosition(iceRobot.SERVO_CENTER);
                    sleep(1000);
                    state += 1;
                    break;
                case 7:
                    iceRobot.encoderMove(50,.75, this);
                    sleep(500);
                    iceRobot.encoderMove(25,.1, this);
                    state += 1;
                    break;
            }


        }




    }
}
