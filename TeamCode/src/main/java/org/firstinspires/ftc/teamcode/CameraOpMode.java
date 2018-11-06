package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="cameraTest",group="Autonomous")
public class CameraOpMode extends LinearOpMode {
    Camera cam = Camera.open(1);
    public void initCamera() {
        cam.setDisplayOrientation(0);
        cam.startPreview();
    }
    public void takePicture() {
        cam.takePicture(Camera.ShutterCallback,
                null,
                null,
                Camera.PictureCallback);
    }
    public void runOpMode(){

    }
}
