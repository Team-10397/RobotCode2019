package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.InvalidClassException;

@Autonomous(name = "Robot: Encoder test!", group = "TeleOp")
public class EncoderTest extends LinearOpMode {
    HardwareRobot iceRobot = new HardwareRobot();

    public void runOpMode(){
        iceRobot.init(hardwareMap);
        telemetry.addData("encoder left:",iceRobot.leftDrive.getCurrentPosition()/iceRobot.encoder_ticks_per_revolution);
        telemetry.addData("encoder right:",iceRobot.rightDrive.getCurrentPosition()/iceRobot.encoder_ticks_per_revolution);

        int state = 0;
        while (true) {
            switch (state) {
                case 0:
                    iceRobot.moveTime(.5,0);
                    state += 1;
                    break;
                case 1:
                    if (iceRobot.leftDrive.getCurrentPosition() > 12*iceRobot.encoder_ticks_per_inch){
                        state+=1;
                    }
                    break;
                case 2:
                    iceRobot.stop();
                    state+=1;
                    break;
            }
        }
    }
}
