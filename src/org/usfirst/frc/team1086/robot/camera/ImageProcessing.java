package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;

public class ImageProcessing {
    CameraServer cs = CameraServer.getInstance();
    public ImageProcessing(){
    }
    public void start(){
        new Thread(() -> {
            AxisCamera camera = CameraServer.getInstance().addAxisCamera("10.10.86.21");
            camera.setResolution(640, 480);
            CameraServer.getInstance().startAutomaticCapture(camera);
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            Mat source = new Mat();
            Mat output = new Mat();
            while(true) {
                cvSink.grabFrame(source);
                openCVProcess(source, output);
                outputStream.putFrame(output);
            }
        }).start();
    }
    public void openCVProcess(Mat source, Mat output){}
}
