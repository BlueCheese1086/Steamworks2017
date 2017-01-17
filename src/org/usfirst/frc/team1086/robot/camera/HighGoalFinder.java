package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDSourceType;
import java.awt.Polygon;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class HighGoalFinder extends CameraCalculator {
    public HighGoalFinder(){
        super(Constants.GEAR_GOAL_HEIGHT);
    }
    @Override public void validateTargets(){
        ArrayList<Polygon> valids = new ArrayList();
        visionObjects.stream().forEach(p -> {
            Rectangle2D bounds = p.getBounds2D();
            double aspectRatio = bounds.getWidth() / bounds.getHeight();
            double area = bounds.getWidth() * bounds.getHeight();
            double solidity = polygonArea(p) / area;
            if (solidity > 0.8 && p.npoints < 20 && aspectRatio > 2) {
                valids.add(p);
            }
        });
        visionObjects = valids;
    }
    @Override public boolean estimationIsGood(){
        return visionObjects.size() == 2 && distance >= 0 && distance <= 240 && Math.abs(angle) < Math.PI / 2;
    }
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }
    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }
}