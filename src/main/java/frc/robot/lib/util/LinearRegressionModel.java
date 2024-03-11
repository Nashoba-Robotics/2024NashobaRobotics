package frc.robot.lib.util;

import java.util.ArrayList;

public class LinearRegressionModel {

    private ArrayList<double[]> points;

    private double slope;
    private double intercept;

    public LinearRegressionModel(ArrayList<double[]> points) {
        this.points = points;
        updateModel();
    }

    // Adds point to the existing model
    public void updateModel(double[] point) {
        points.add(point);
        updateModel();
    }

    private void updateModel() {
        double xMean = 0;
        double yMean = 0;
        for(double[] point : points) {
            xMean += point[0];
            yMean += point[1];
        }
        xMean /= points.size();
        yMean /= points.size();

        double sy = 0;
        double sx = 0;
        for(double[] point : points) {
            double xDiff = point[0] - xMean;
            double yDiff = point[1] - yMean;
            sy += xDiff * yDiff;
            sx += xDiff * xDiff;
        }

        slope = sy/sx;

        intercept = yMean - xMean * slope;
    }

    public void resetModel(ArrayList<double[]> points) {
        this.points = points;
        updateModel();
    }

    public ArrayList<double[]> getPoints() {
        return points;
    }

    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }
    
}
