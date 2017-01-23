package org.usfirst.frc.team1086.robot.camera;

import java.util.ArrayList;
import org.opencv.core.MatOfPoint;

public interface CVDataHandler {
    public void handle(ArrayList<MatOfPoint> m);
}
