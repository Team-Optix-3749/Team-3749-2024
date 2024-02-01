package frc.robot.subsystems.arm;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj.Filesystem;

// TODO: Clean up this code, abstract the code and chunk it out, check if logic works on global level (sim or smth)

public class ShootKinematics {
    // constants move to constants file
    private static final Translation2d redSpeakerPosition = new Translation2d(16.591, 5.553); // rounded need to change
    private static final Translation2d blueSpeakerPosition = new Translation2d(0, 5.553); // rounded need to change
    
    // Note: some of these points are basically not used, tolerances of 10 inches were used
    private static final Translation2d[] redStagePoints = {new Translation2d(13.8986, 4.1056), new Translation2d(10.4648, 2.1752),new Translation2d(10.4648, 6.0361)};
    private static final Translation2d[] blueStagePoints = {new Translation2d(16.5928-redStagePoints[0].getX(), redStagePoints[0].getY()),new Translation2d(16.5928-redStagePoints[1].getX(), redStagePoints[1].getY()),new Translation2d(16.5928-redStagePoints[2].getX(), redStagePoints[2].getY())};

    // 10.00 m = 1000
    // angle 0.0 = impossible to shoot from here
    private static final double[] distToAngle = new double[1001];
    private static double maxDist = 0.0;

    public static Pose2d shootingPose2DCalculate(Pose2d currentPose2d){
        Rotation2d angle;

        Translation2d distanceVector = currentPose2d.getTranslation().minus(getSpeakerPosition());

        angle = new Rotation2d(Math.PI/2 - Math.atan2(Math.abs(distanceVector.getY()), Math.abs(distanceVector.getX())));

        //double distAngle = getAngle(distanceVector.getNorm());
        
        // Case 0: We are in angle
        if (angle.getDegrees() > Constants.ArmConstants.maxAngle && distanceVector.getNorm() <= maxDist){ 
            return moveOutOfStage(changeRotation(currentPose2d.getTranslation(), distanceVector));
        } 
        // Case 1: We are out of angle
        if (angle.getDegrees() <= Constants.ArmConstants.maxAngle) {

            // TODO: Check if positive/negative x coord check is correct
            Translation2d radiusVector;

            if (distanceVector.getX() > 0) {
                radiusVector = new Translation2d(Math.cos(Constants.ArmConstants.maxAngleRad), Math.sin(Constants.ArmConstants.maxAngleRad));
            } else {
                radiusVector = new Translation2d(Math.cos(-Constants.ArmConstants.maxAngleRad), Math.sin(-Constants.ArmConstants.maxAngleRad));
            }
 
            Translation2d perpVector = projection(distanceVector, radiusVector).minus(distanceVector);
            Translation2d goal = perpVector.plus(currentPose2d.getTranslation());

            // Case 3: We are out of range and out of angle
            Translation2d newDistanceVector = goal.minus(getSpeakerPosition());
            if (newDistanceVector.getNorm() > maxDist) {
                goal = getSpeakerPosition().plus(newDistanceVector.div(newDistanceVector.getNorm()).times(maxDist));
            }

            return moveOutOfStage(changeRotation(goal, goal.minus(getSpeakerPosition())));
        }
        // Case 2: We are out of range
        if (distanceVector.getNorm() > maxDist) {
            Translation2d goal = getSpeakerPosition().plus(distanceVector.div(distanceVector.getNorm()).times(maxDist));
            return moveOutOfStage(changeRotation(goal, goal.minus(getSpeakerPosition())));
        }

        return null;
    }

    private static Pose2d changeRotation(Translation2d currentTranslation2d, Translation2d distanceVector){
        return new Pose2d(currentTranslation2d, new Rotation2d(-distanceVector.getAngle().getRadians()));
    }

    // Case 5 Check if we are in stage and move accordingly
    private static Pose2d moveOutOfStage(Pose2d poseInRadius){
        Translation2d[] stagePoints = getStagePoints();

        Translation2d distanceVector = poseInRadius.getTranslation().minus(stagePoints[0]);
        double angle = Math.abs(distanceVector.getAngle().getDegrees());

        if (angle < Math.PI/6 && poseInRadius.getTranslation().getX() < stagePoints[1].getX()){
            Translation2d perpVector = projection(distanceVector, stagePoints[2].minus(stagePoints[0]).minus(distanceVector));
            Translation2d nearestShootPoint = poseInRadius.getTranslation().plus(perpVector);
            
            return changeRotation(nearestShootPoint, nearestShootPoint.minus(getSpeakerPosition()));
        }

        return poseInRadius;
    }

    private static Translation2d projection(Translation2d vector, Translation2d target) {
        return target.div(Math.pow(target.getNorm(),2)).times(dotProduct(target, vector));
    }

    private static double dotProduct(Translation2d v1, Translation2d v2) {
        return v1.getX() * v2.getX() + v1.getY() * v2.getY();
    }

    private static double getAngle(double dist) {
        int distNum = (int)(Math.round(dist * 100.0));
        if (distNum < 0 || distNum > 1000) {
            return 0;
        }
        return distToAngle[distNum];
    }

    private static Translation2d getSpeakerPosition() {
        try {
            return (DriverStation.getAlliance().get() == Alliance.Red) ? redSpeakerPosition : blueSpeakerPosition;
        } catch (Exception e) {
            return blueSpeakerPosition;
        }
    }

    private static Translation2d[] getStagePoints(){
        try {
            return (DriverStation.getAlliance().get() == Alliance.Red) ? redStagePoints : blueStagePoints;
        } catch (Exception e) {
            return blueStagePoints;
        }
    }

    public static void loadDistCSV() throws FileNotFoundException, IOException {
        Path csvPath = Filesystem.getDeployDirectory().toPath().resolve("angles.csv");
        loadDistCSV(csvPath.toFile());
    }

    public static void loadDistCSV(File file) throws FileNotFoundException, IOException {
        BufferedReader reader = new BufferedReader(new FileReader(file));
        
        String line;
        while ((line = reader.readLine()) != null) {
            String[] values = line.split(",");
            double curDist = Double.parseDouble(values[0]);
            maxDist = Math.max(maxDist, curDist);
            distToAngle[(int)(curDist * 100)] = Double.parseDouble(values[1]);
        }

        maxDist -= Constants.ArmConstants.distMargin;

        reader.close();
    }

    // for testing load csv & other functionality
    public static void main(String[] args) throws FileNotFoundException, IOException {
        loadDistCSV(new File("src/main/deploy/angles.csv"));
        for (double i = 0.9; i < maxDist; i += 0.01) {
            i = Math.round(i*100)/100.0;
            double temp = i + Math.random()*.01;
            temp = Math.round(temp*1000)/1000.0;
            System.out.println(temp + " " + getAngle(temp));
        }
    }
}
