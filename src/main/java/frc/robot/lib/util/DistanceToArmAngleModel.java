package frc.robot.lib.util;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class DistanceToArmAngleModel {
    
    private ArrayList<double[]> untransformedPoints;
    private ArrayList<double[]> transformedPoints;
    private LinearRegressionModel distanceToAngleRegressionModel;

    private double deleteRange;

    public double lastDistanceToShoot;


    private static HashMap<String, DistanceToArmAngleModel> instanceMap;

    public static DistanceToArmAngleModel getInstance(String fileName) {
        if(instanceMap == null) instanceMap = new HashMap<>();
        
        if(!instanceMap.containsKey(fileName)) {
            newInstance(fileName);
        }

        return instanceMap.get(fileName);
    }

    public static DistanceToArmAngleModel newInstance(String fileName) {
        if(instanceMap == null) instanceMap = new HashMap<>();

        try {
            String ext = fileName.split("\\.")[1];
            if(ext.equals("pts")) {
                instanceMap.put(
                    fileName,
                    new DistanceToArmAngleModel(new File(Filesystem.getDeployDirectory().getPath() + "/regression/" + fileName), Constants.Misc.DELETE_DISTANCE_RANGE)
                );
            } else if(ext.equals("eq")) {
                Scanner s = new Scanner(new File(Filesystem.getDeployDirectory().getPath() + "/regression/" + fileName));
                String line = s.nextLine();
                s.close();
                String[] tokens = line.split(" ");
                double slope = Double.parseDouble(tokens[2]);
                double intercept = Double.parseDouble(tokens[6]);
                instanceMap.put(
                    fileName,
                    new DistanceToArmAngleModel(slope, intercept, Constants.Misc.DELETE_DISTANCE_RANGE)
                );
            }
        } catch(Exception e) {
            instanceMap.put(
                fileName, 
                new DistanceToArmAngleModel(new ArrayList<>(), Constants.Misc.DELETE_DISTANCE_RANGE)
            );
        }

        return instanceMap.get(fileName);
    }

    public DistanceToArmAngleModel(ArrayList<double[]> distanceToAnglePoints, double deleteRange) {
        lastDistanceToShoot = 0;
        this.untransformedPoints = distanceToAnglePoints;
        this.transformedPoints = new ArrayList<>();

        this.deleteRange = deleteRange;

        for(double[] point : distanceToAnglePoints) {
            transformedPoints.add(transformPoint(point));
        }

        distanceToAngleRegressionModel = new LinearRegressionModel(transformedPoints);
    }

    public DistanceToArmAngleModel(File distanceToAnglePointsFile, double deleteRange) {
        lastDistanceToShoot = 0;
        this.untransformedPoints = new ArrayList<>();
        this.transformedPoints = new ArrayList<>();

        this.deleteRange = deleteRange;

        try {
            Scanner s = new Scanner(distanceToAnglePointsFile);

            while(s.hasNextLine()) {
                String[] nums = s.nextLine().split(" ");
                untransformedPoints.add(new double[] {Double.parseDouble(nums[0]), Double.parseDouble(nums[1])});
                transformedPoints.add(transformPoint(new double[] {Double.parseDouble(nums[0]), Double.parseDouble(nums[1])}));
            }

            s.close();
        } catch(Exception e) {
            System.out.println("Whoops: " + e);
        }

        distanceToAngleRegressionModel = new LinearRegressionModel(transformedPoints);
    }

    public DistanceToArmAngleModel(double slope, double intercept, double deleteRange) {
        lastDistanceToShoot = 0;
        this.untransformedPoints = new ArrayList<>();
        this.transformedPoints = new ArrayList<>();
        this.deleteRange = deleteRange;

        for(double i = 1.5; i <= 5.00; i+= 0.5) {
            double x = Math.atan(i);
            untransformedPoints.add(new double[] {i, slope*x + intercept});
            transformedPoints.add(new double[] {x, slope*x + intercept});
        }

        distanceToAngleRegressionModel = new LinearRegressionModel(transformedPoints);
    }

    public void updateModel(double[] point, boolean deleteClosestInRange) {
        if(deleteClosestInRange) {
            int indexOfLowestValue = -1;
            double minDist = Double.MAX_VALUE;

            for(int i = 0; i < untransformedPoints.size(); i++) {
                double dist = Math.abs(point[0] - untransformedPoints.get(i)[0]);
                if(dist < deleteRange && dist < minDist) {
                    indexOfLowestValue = i;
                    minDist = dist;
                }
            }

            if(indexOfLowestValue != -1) {
                untransformedPoints.remove(indexOfLowestValue);
                transformedPoints.remove(indexOfLowestValue);
            }
        }

        untransformedPoints.add(point);
        transformedPoints.add(transformPoint(point));

        resetModel();
    }

    public void updateModel(ArrayList<double[]> points) {
        untransformedPoints = points;
        for(double[] point : points) {
            transformedPoints.add(transformPoint(point));
        }
        resetModel();
    }

    public void resetModel() {
        distanceToAngleRegressionModel.resetModel(transformedPoints);
    }

    private double[] transformPoint(double[] point) {
        return new double[] {Math.tan(point[0]), point[1]};
    }

    private double[] untransformPoint(double[] point) {
        return new double[] {Math.tan(point[0]), point[1]};
    }

    public double applyFunction(double x) {
        x = Math.atan(x);
        return x * distanceToAngleRegressionModel.getSlope() + distanceToAngleRegressionModel.getIntercept();
    }

    public double getSlope() {
        return distanceToAngleRegressionModel.getSlope();
    }

    public double getIntercept() {
        return distanceToAngleRegressionModel.getIntercept();
    }

    public ArrayList<double[]> getTransformedPoints() {
       return transformedPoints;
    }

    public ArrayList<double[]> getUntransformedPoints() {
        return untransformedPoints;
    }

    public String getEquation() {
        return "y = " + getSlope() + " * x + " + getIntercept();
    }

}
