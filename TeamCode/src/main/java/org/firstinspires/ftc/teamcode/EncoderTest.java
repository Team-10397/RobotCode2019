package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Robot: Encoder test!", group = "TeleOp")
public class EncoderTest extends OpMode {
    HardwareRobot iceRobot = new HardwareRobot();

    public void init(){
        iceRobot.init(hardwareMap);
    }
    public void loop(){

        telemetry.addData("encoder left:",iceRobot.leftDrive.getCurrentPosition()/iceRobot.encoder_ticks_per_revolution);
        telemetry.addData("encoder right:",iceRobot.rightDrive.getCurrentPosition()/iceRobot.encoder_ticks_per_revolution);
        if (gamepad1.dpad_up) {
            iceRobot.encoderMove(12);
        }
        if (gamepad1.dpad_down) {
            iceRobot.encoderMove(-12);
        }
        if (gamepad1.dpad_right) {
            iceRobot.encoderTurn(90);
        }
        if (gamepad1.dpad_left) {
            iceRobot.encoderTurn(-90);
        }
    }
}
