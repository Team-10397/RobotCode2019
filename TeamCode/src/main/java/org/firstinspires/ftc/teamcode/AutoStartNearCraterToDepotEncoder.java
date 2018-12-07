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

@Autonomous(name="Autonomous Crater To Depot With Encoder", group="Autonomous")
public class AutoStartNearCraterToDepotEncoder extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    HardwareRobot iceRobot = new HardwareRobot();
    int state = 0; // TODO: use enums

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        iceRobot.init(hardwareMap);

        waitForStart();

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
                    iceRobot.encoderMove(-20,1, this);
                    sleep(500);
                    state += 1;
                    break;
                case 5:
                    iceRobot.encoderTurn(-90);
                    sleep(500);
                    iceRobot.encoderMove(-40,.8,this);
                    sleep(500);
                    iceRobot.encoderTurn(45);
                    sleep(500);
                    iceRobot.encoderMove(-50,1,this);
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
