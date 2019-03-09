package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Camera {

    static final double pixelPerDegreeConstant = 0.146875;
    static final double offsetConstant = 10;
    static final double FIELD_OF_VIEW_RAD = 70.42 * Math.PI /180.0;
    static final double FOCAL_LENGTH_PIXELS = (640 / 2) / Math.tan(FIELD_OF_VIEW_RAD / 2.0);
    
    public int width = 0;
    public double distance = 0;
    public double distToCenter = 0;
    public double angle = 0;
    public boolean firstRun = true;
    public int visionDirection = 0;
    int numValid = 0;
    static NetworkTableInstance inst;
    static NetworkTable visionTable;
    private static NetworkTableEntry visionStringArrEntry;
    public boolean dumbVision = false;

    public boolean turnConfirmed = false;
    public boolean turningRight = false;

    public static final double A = -1.029258435778039;
    public static final double B = 0.12131357724670491;
    public static final double C = -0.0004412063928563716;
    public static final double D = 7.653889638753478e-7;
    
    public Camera() {
        inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("JetsonTable"); 
        visionStringArrEntry = visionTable.getEntry("String Array");
    }
    
    public void updateVision() {
     
        if (visionDirection == 2 && getDesiredHeading(false) != 0 & !turnConfirmed) {
            if (distToCenter > 0) {
                //start to the left and turn right
                turnConfirmed = true;
                turningRight = true;
            } else if (distToCenter < 0) {
                //start to the right and turn left
                turningRight = false;
                turnConfirmed = true;
            }
        }


        if (turnConfirmed && numValid == 2) {
            if (turningRight) {visionDirection = 1; } 
            else {visionDirection = 3; }
        }

        if (!dumbVision) {
            visionDirection = 0;
        }

        int[][] visionArr = getVisionContours(visionDirection, visionStringArrEntry.getStringArray(new String[0]));
        
        if (visionArr.length == 2 && (visionArr[1][0] - visionArr[0][1] != 0)) {
            double centerX = (visionArr[0][1] + visionArr[1][0]) /2;
            centerX = centerX + offsetConstant;
            width = visionArr[1][0] - visionArr[0][1];
            distToCenter = centerX - 320;
            distance = 4400 / width;
            angle = Math.toDegrees(Math.atan(distToCenter / FOCAL_LENGTH_PIXELS));
            SmartDashboard.putBoolean("Contour Found", true);
        } else {
            //We are not getting contours
            angle = 0;
            SmartDashboard.putBoolean("Contour Found", false);
        }   
    }
    
    public int getDesiredHeading(boolean turnLess) {
        int[] testHeadings = {0,-90, 90, -61, -151, 61, 151, 180, -180};
        int leastHeadingIndex = 0;
        double leastHeading = 300;
        double currentHeading = Robot.navX.getYaw();
        double avoidHeading = 5000000;
        
        if (turnLess) {
            avoidHeading = 500000;
        }

        for (int i = 0; i < testHeadings.length; i++) {
            double testHeading = testHeadings[i];
            double headingDifference = Math.abs(testHeading - currentHeading);
            if ((headingDifference < leastHeading) && (Math.abs(testHeading) != avoidHeading)) {
                leastHeading = headingDifference;
                leastHeadingIndex = i;
            }
        }
        return testHeadings[leastHeadingIndex];
    }

    public double getCameraDegreeOffset() {
        return angle;
    }

    public double getCentDist() {
        return distToCenter;
    }

    public double getWidth() {
        return width;
    }

    public double getDistance() {
        return distance;
    }
    
    public double getPixelOffset(double differenceInHeading) {
        //Offset from target 
        double inchesOffset = distance * Math.tan(Math.toRadians(differenceInHeading));     
        double pixelOffset = inchesOffset * (width / 8);
        return pixelOffset;
    }

    public int[][] getVisionContours(int position, String[] inputArr) { 
        
        ArrayList<String> nextList = new ArrayList<>();

        for (String cont : inputArr) {
            if (!cont.isBlank()) {
                nextList.add(cont);
            }
        }

        String[] nextArr = new String[nextList.size()];
        for (int i = 0; i < nextList.size(); i++) {
            nextArr[i] = nextList.get(i);
        }

        int[][] visionArr = convtVisionArr(nextArr);
        visionArr = sortArr(visionArr);
        
        return calculate(visionArr, position);
    }
    
    public int[][] convtVisionArr(String[] arr){
        int[][] finalArr = new int[arr.length][6];

        int outerIndex = 0;
        for (String conts : arr){
            int innerIndex = 0;
            for (String val : conts.split(" ")){
                int tempVal = Integer.parseInt(val);
                finalArr[outerIndex][innerIndex] = tempVal; 
                innerIndex++;
            }
            
            outerIndex++;
        }
        
        return finalArr;
    }

    public int[][] sortArr(int[][] arr){

        ArrayList <int[][]>valid = new ArrayList <int[][]>();
        
        for(int i = 0; i< (arr.length - 1); i++) {
            if(individualContourValid(arr[i])) {
                valid.add(new int[][] {arr[i]});
            }
        }

        SmartDashboard.putNumber("IND Valid Contours", valid.size());
        
        Arrays.sort(arr, new Comparator<int[]>() {
            public int compare(int[] entry1, int[] entry2) {
                final int val1 = entry1[0];
                final int val2 = entry2[0];
                
                if (val1 < val2) {
                    return -1;
                }
                else {
                    return 1;
                }
            }
        });

        return arr; 
    }


    public boolean individualContourValid(int[] arr) {
        //[ul.x, ur.x, ll.x, lr.x, ul.y, ll.y]

        double width = 0;
        if (arr[0] < arr[1]) {
            //lower right - upper left
            width = arr[3] - arr[0];
        } else {
            width = arr[1] - arr[2];
        }

        double height = arr[5] - arr[4];
        double ratio = width/height;
        SmartDashboard.putNumber("Ratio", ratio);
        double correctRatio = 2/5.5;   
    

        return Math.abs(ratio - correctRatio) <= 0.5;

    }

    public int[][] calculate(int[][] arr, int position) {

	    ArrayList <int[][]>validArray = new ArrayList <int[][]>();
        
        for(int i = 0; i< (arr.length - 1); i++) {
            if(isValidCont(arr[i], arr[i+1])) {
                validArray.add(new int[][] {arr[i], arr[i + 1]});
            }
        }
        
        numValid = validArray.size();
		

		if (validArray.size() > 0) {

            if (validArray.size() == 1) {
                return validArray.get(0);
            }

	        double leastDifference = 400;
            int leastDifferenceIndex = 0;
		    int index = 0;
		
            if (position == 0 && visionDirection == 0) {
                for (int i = 0; i < (arr.length - 1) ; i++) {
                    if (isValidCont(arr[i], arr[i + 1])) {
                        index++;
                        double centerX = (arr[i][1] + arr[i + 1][0]) /2;
                        double centDist = Math.abs(centerX - 320);
                        if (centDist < leastDifference) {
                            leastDifference = centDist;
                            leastDifferenceIndex = i;
                            visionDirection = index;
                        }
                    }
                }

                if (validArray.size() == 2 && visionDirection == 2) {
                    visionDirection = 3;
                }

                return new int[][] {arr[leastDifferenceIndex], arr[leastDifferenceIndex + 1]};
            } else {
               int validIndex = 0;
               for (int i = 0; i < arr.length - 1; i++) {
                 if (isValidCont(arr[i], arr[i+1])){ 
                        validIndex++;
                        if (position == 1) {
                            return validArray.get(0);                                       
                        } 
                        else if (position == 2) {
                            if (validIndex == 2) {
                                return new int[][] {arr[i], arr[i + 1]};
                            }   
                        } else if (position == 3) {
                            if (validIndex == 3) {
                                return validArray.get(validArray.size() - 1);
                            }   
                        }
                    }
                }
		    }
	    }

        return new int[][] {};
    }

    // needs to test deciding the distance between ul and ur is less than ll,lr --> get a set of contours that are valid
    public boolean isValidCont(int[] cont1, int[] cont2){
        int ul1 = cont1[0], ur1 = cont1[1], ll1 = cont1[2], lr1 = cont1[3];
        int ul2 = cont2[0], ur2 = cont2[1], ll2 = cont2[2], lr2 = cont2[3];  

        int topDiff = ul2 - ur1;
        int botDiff = ll2 - lr1;

        return topDiff < botDiff; // quiets the compiler
    }
    

}