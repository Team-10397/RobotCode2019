package org.firstinspires.ftc.teamcode;

import android.content.Intent;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.os.Bundle;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static android.app.Activity.RESULT_OK;

@Autonomous(name="cameraTest",group="Autonomous")
public class CameraOpMode extends LinearOpMode {
    Camera cam = Camera.open(1);
    static final int REQUEST_IMAGE_CAPTURE = 1;
    public void takePicture() {

        private void dispatchTakePictureIntent(){
            Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
            if (takePictureIntent.resolveActivity(getPackageManager()) != null) {
                startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE);
            }
        }
        @Override
        protected void onActivityResult ( int requestCode, int resultCode, Intent data){
            if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == RESULT_OK) {
                Bundle extras = data.getExtras();
                Bitmap imageBitmap = (Bitmap) extras.get("data");
                telemetry.addData(toString(bitmap.getPixel(50,50)));
            }
        }
    }

    public void runOpMode(){
        takePicture();
    }
}
