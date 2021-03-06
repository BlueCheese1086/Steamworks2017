package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ImageProcessing {
    CameraServer cs = CameraServer.getInstance();
    CVDataHandler handler;
    Mat hsvThresholdOutput = new Mat();
    int tick = 0;
    boolean go = true;
    ArrayList<MatOfPoint> findContoursOutput = new ArrayList();
    ArrayList<MatOfPoint> filterContoursOutput = new ArrayList();
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    public ImageProcessing(){
    }
    public void start(){
        new Thread(){
        	@Override public void run(){
                AxisCamera camera1 = CameraServer.getInstance().addAxisCamera("10.10.86.22");
                AxisCamera camera2 = CameraServer.getInstance().addAxisCamera("10.10.86.23");
                camera1.setResolution(320, 240);
                try {
	                /*while(!camera.isConnected()){
	                	System.out.println("Camera not connected yet... retrying in 1 second");
	                	sleep(1000);
	                	System.out.println(camera.getName());
	                	System.out.println(camera.isConnected());
	                }
	                System.out.println("Stacey, the camera is connected now. You can restart.");*/
	                CameraServer.getInstance().startAutomaticCapture(camera1);
	                CameraServer.getInstance().startAutomaticCapture(camera2);
	                CvSink cvSink = CameraServer.getInstance().getVideo();
	                cvSink.setSource(camera1);
	                Mat source = new Mat();
	                Mat output = new Mat();
	                while (!interrupted() && go) {
	                	cvSink.grabFrame(source);
	                	openCVProcess(source, output);
		                sleep(40);
		                Runtime.getRuntime().gc();
	                }
                } catch (Exception e){}
            }
        }.start();
    }
    public void stop(){
    	go = false;
    }
    public void setCameraTarget(CVDataHandler handler){
        this.handler = handler;
    }
    public void openCVProcess(Mat source, Mat output){
        //HSV
        Mat hsvThresholdInput = source;
        double[] hsvThresholdHue = {49, 95};
        double[] hsvThresholdSaturation = {231, 255};
        double[] hsvThresholdValue = {30, 248};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, output);

        hsvThresholdOutput = output.clone();

        //GET CONTOURS
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        //FILTER CONTOURS
        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
        double filterContoursMinArea = 50;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 0;
        double filterContoursMaxWidth = 700.0;
        double filterContoursMinHeight = 0;
        double filterContoursMaxHeight = 1000;
        double[] filterContoursSolidity = {30, 100.0};
        double filterContoursMaxVertices = 1000000;
        double filterContoursMinVertices = 0;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 1000;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
        handler.handle(filterContoursOutput);
    }
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out){
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }
    private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours){
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if(externalOnly){
            mode = Imgproc.RETR_EXTERNAL;
        } else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
    private void filterContours(List<MatOfPoint> inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, List<MatOfPoint> output){
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) {
                continue;
            }
            if (bb.height < minHeight || bb.height > maxHeight) {
                continue;
            }
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) {
                continue;
            }
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) {
                continue;
            }
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) {
                continue;
            }
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount) {
                continue;
            }
            final double ratio = bb.width / (double) bb.height;
            if (ratio < minRatio || ratio > maxRatio) {
                continue;
            }
            output.add(contour);
        }
    }
}