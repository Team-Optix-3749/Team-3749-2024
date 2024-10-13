package frc.robot.subsystems.arm;

import java.io.*;

import edu.wpi.first.math.util.Units;

// NOTE: all distances in output file are relative to center of robot, arm offset is accounted for
public class ArmAngleCalculator {
    public static double y_speaker = Units.inchesToMeters(79); //MOVED FROM ARM CONSTANTS
    public static double x_arm = -0.2286; // how far arm is from center of robot // MOVED FROM ARM CONSTANTS
    public static double y_arm = Units.inchesToMeters(12.75); // how high up the arm is //MOVED FROM ARM CONSTANTS
    public static double v_initial = 12.5; // MOVED FROM ARM CONSTANTS

    public static double arm_length = 0.61;

    // Gravity
    public static final double g = 9.81;

    // Range of Angles & Minimum Shooting Distance
    public static final double initial_angle = 90.0;
    public static final double final_angle = 0.00;
    public static double min_distance = 0.9;

    // Differentials & Margins of Error
    public static final double angle_increment = 0.01;
    public static final double dist_increment = 0.01;
    public static final double margin_of_error = 0.001;

    public static void main(String[] args) throws IOException, Exception {
        double previous_angle = 0;
        double current_angle = 0.01;
        String csv = "";

        // if angle stops increasing, then shooting downwards (STOP)
        for (double i = min_distance; current_angle > previous_angle; i = round(i + dist_increment)) {
            double angle = calculateAngle(i);
            csv += i + "," + angle + "\n";
            
            previous_angle = current_angle;
            current_angle = angle;
        }

        // output CSV to file
        PrintWriter pw = new PrintWriter("src/main/java/frc/robot/subsystems/arm/angles.csv");
        pw.print(csv);
        pw.close();

        PrintWriter pw2 = new PrintWriter("src/main/deploy/angles.csv");
        pw2.print(csv);
        pw2.close();
    }

    public static double calculateAngle(double x_dist) throws Exception {
         // if angles decrease then it is shooting downwards (STOP)
         for (double i = initial_angle; i >= final_angle; i = round(i - angle_increment)) {
               double initial_angle_rad = Math.toRadians(i);
               double shoot_angle_rad = Math.toRadians(60-i);
               
               double vx = Math.cos(shoot_angle_rad) * v_initial;
               double t = ( x_dist + x_arm + arm_length*Math.cos(initial_angle_rad) ) / vx;
               double y = ( Math.sin(shoot_angle_rad) * v_initial * t ) - ( g/2 * t*t ) + ( arm_length * Math.sin(initial_angle_rad) ) + y_arm;

               if (Math.abs(y_speaker - y) <= margin_of_error) {
                    System.out.print("x-dist: ");
                    System.out.println(x_dist);

                    System.out.print("y-value: ");
                    System.out.println(y);

                    System.out.print("arm angle: ");
                    System.out.println(Math.toDegrees(initial_angle_rad));

                    System.out.print("shooter angle: ");
                    System.out.println(Math.toDegrees(shoot_angle_rad));

                    System.out.println("---------------------------");
                    return i;
               }
         }

         throw new Exception("No angle for distance " + x_dist);
    }
    
    
    public static double round(double num) {
        return Math.round(num * 100) / 100.0;
    }
}
