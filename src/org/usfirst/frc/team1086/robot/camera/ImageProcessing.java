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
   
    Mat hsvThresholdOutput = new Mat();
    ArrayList<MatOfPoint> findContoursOutput = new ArrayList();	
    ArrayList<MatOfPoint> filterContoursOutput = new ArrayList();
    
    static {
    	System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
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
                //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();
    }
    public void openCVProcess(Mat source, Mat output){
    	//HSV
    	Mat hsvThresholdInput = source;
    	double[] hsvThresholdHue = {67.98561151079137, 126.24573378839587};
		double[] hsvThresholdSaturation = {4.586330935251798, 117.92662116040952};
		double[] hsvThresholdValue = {126.12410071942448, 255.0};
    	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, output);
    	
    	hsvThresholdOutput = output.clone();
    	
    	//GET CONTOURS
    	Mat findContoursInput = hsvThresholdOutput;
    	boolean findContoursExternalOnly = false;
    	findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
    
    	//FILTER CONTOURS
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 75.0;
		double filterContoursMinPerimeter = 75.0;
		double filterContoursMinWidth = 50.0;
		double filterContoursMaxWidth = 700.0;
		double filterContoursMinHeight = 0;
		double filterContoursMaxHeight = 1000;
		double[] filterContoursSolidity = {9.892086330935252, 100.0};
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = 0;
		double filterContoursMaxRatio = 1000;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
		
		//TO-DO: Output filterContoursOutput to NetworkTables
    }
    
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}
    

	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}
	
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
			double minPerimeter, double minWidth, double maxWidth, double minHeight, double
			maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
			minRatio, double maxRatio, List<MatOfPoint> output) {
			final MatOfInt hull = new MatOfInt();
			output.clear();
			//operation
			for (int i = 0; i < inputContours.size(); i++) {
				final MatOfPoint contour = inputContours.get(i);
				final Rect bb = Imgproc.boundingRect(contour);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				final double area = Imgproc.contourArea(contour);
				if (area < minArea) continue;
				if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
				Imgproc.convexHull(contour, hull);
				MatOfPoint mopHull = new MatOfPoint();
				mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
				for (int j = 0; j < hull.size().height; j++) {
					int index = (int)hull.get(j, 0)[0];
					double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
					mopHull.put(j, 0, point);
				}
				final double solid = 100 * area / Imgproc.contourArea(mopHull);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
				final double ratio = bb.width / (double)bb.height;
				if (ratio < minRatio || ratio > maxRatio) continue;
				output.add(contour);
			}
		}
}
