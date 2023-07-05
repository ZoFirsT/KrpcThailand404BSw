package jp.jaxa.iss.kibo.rpc.Thailand;

import android.util.Log;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.core.Rect;

import org.opencv.objdetect.QRCodeDetector;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private static final double INF = Double.POSITIVE_INFINITY; // an infinity value to indicate disconnection
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        final long MINIMUM_TIME_REMAINING = 60 * 1000; // 1 minute
        final String MISSION_START_LOG = "Mission started";
        final String GOAL_REACHED_LOG = "Moving to goal";

        try {
            // Start the mission
            api.startMission();
            Log.i(TAG, MISSION_START_LOG);

            // Move to initial point
            MoveToWaypoint(waypoints_config.wp1); // initial point

            // Move to QR code scanning point
            MoveToWaypoint(waypoints_config.wp2); // QR point
            Global.Nowplace = 8;

            // Scan QR code
            Mat image = api.getMatNavCam();
            if (image != null) {
                api.saveMatImage(image,"wp2.png");
                String report = readQRCode(image);

                if (report != null && !report.isEmpty()) {
                    // Search active targets until less than minimum time remains
                    while (api.getTimeRemaining().get(1) > MINIMUM_TIME_REMAINING) {
                        Log.i(TAG, "Current position in runPlan1: " + Global.Nowplace);
                        GoTarget(api.getActiveTargets());
                    }

                    // Go to goal when minimum time remains
                    Log.i(TAG, GOAL_REACHED_LOG);
                    MoveToWaypoint(waypoints_config.goal_point);

                    // Notify completion of mission
                    api.notifyGoingToGoal();
                    api.reportMissionCompletion(report);
                } else {
                    Log.e(TAG, "QR Code reading returned null or empty report");
                }
            } else {
                Log.e(TAG, "Failed to capture image from NavCam");
            }
        } catch (Exception e) {
            Log.e(TAG, "Error running plan 1", e);
        }
    }



    @Override
    protected void runPlan2(){
       // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){
        final Point point = new Point((float)pos_x, (float)pos_y, (float)pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        api.moveTo(point, quaternion, true);
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {
        final Point point = new Point((float)pos_x, (float)pos_y, (float)pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        Result result = api.relativeMoveTo(point, quaternion, true);

        // Output whether the movement command is operating normally with the Status enum type For details, please refer to the following URL
        // https://github.com/nasa/astrobee_android/blob/a8560ab0270ac281d8eadeb48645f4224582985e/astrobee_api/api/src/main/java/gov/nasa/arc/astrobee/Result.java
        if(result.hasSucceeded()){
            String str = result.getStatus().toString();
            Log.i(TAG, "[relativeMoveToWrapper]:"+str);
        }else{
            Log.w(TAG, " api.relativeMoveTo Error : result.hasSucceeded()=false");
        }
    }


    private void MoveToWaypoint(Waypoint name){

        final int LOOP_MAX = 10;

        int count = 0;
        while(count < LOOP_MAX){
            final Point point = new Point(
                    (float)(name.posX + name.avoidX*count),
                    (float)(name.posY + name.avoidY*count),
                    (float)(name.posZ + name.avoidZ*count) );

            final Quaternion quaternion = new Quaternion(
                    (float)name.quaX,
                    (float)name.quaY,
                    (float)name.quaZ,
                    (float)name.quaW
                        );

            Result result = api.moveTo(point, quaternion, true);
            ++count;

            if(result.hasSucceeded()){
                break;
            }
            Log.w(TAG, "move Failure, retry");
        }
    }

    private void Print_AR(List<Mat> corners, Mat markerIds) {
        for (int n = 0; n < 4; n++) {
            Log.i(TAG, "markerIds:" + Arrays.toString(markerIds.get(n,0)));
            Log.i(TAG, "Top Left:" + Arrays.toString(corners.get(n).get(0, 0)));
            Log.i(TAG, "Top Right:" + Arrays.toString(corners.get(n).get(0, 1)));
            Log.i(TAG, "Bottom Right:" + Arrays.toString(corners.get(n).get(0, 2)));
            Log.i(TAG, "bottom left:" + Arrays.toString(corners.get(n).get(0, 3)));
        }
    }

    // find the bottom right marker
    private int findBottomRight(List<Mat> corners){
        Log.i(TAG,"start findBottomRight");
        // out = function return
        int out = 0;
        int temp = 0;

        //corners.get(n).get(0, 0) -> get the lower right xy coordinates of the nth marker
        for(int n=0; n<4; n++){
            Log.i(TAG,"Loop" + n );
            // Use the Pythagorean theorem that the largest number is the farthest
            // a^2 + b^2 = c^2
            double[]ab = corners.get(n).get(0,2);
            int c = (int)ab[0] * (int)ab[0] + (int)ab[1] * (int)ab[1];
            if(temp < c ){
                temp = c;
                out = n;
                Log.i(TAG,"change");
            }
        }
        // Returns the number of the bottom right (farthest) in the array
        Log.i(TAG,"finish findBottomRight");
        return out;
    }

    // Kinematics Github
    // https://github.com/nasa/astrobee_android/blob/a8560ab0270ac281d8eadeb48645f4224582985e/astrobee_api/api/src/main/java/gov/nasa/arc/astrobee/Kinematics.java
    private void LoggingKinematics(){
        //Kinematics no Log
        Kinematics kinematics = api.getRobotKinematics();
        Log.i(TAG, "[LoggingKinematics]: state" + kinematics.getConfidence().toString());
        Log.i(TAG, "[LoggingKinematics]: absolute coordinates" + kinematics.getPosition().toString());
        Log.i(TAG, "[LoggingKinematics]: orientation coordinate" + kinematics.getOrientation().toString());
        Log.i(TAG, "[LoggingKinematics]: Linear Velocity" + kinematics.getLinearVelocity().toString());      // linear velocity
        Log.i(TAG, "[LoggingKinematics]: angular velocity" + kinematics.getAngularVelocity().toString()); // angular velocity
        Log.i(TAG, "[LoggingKinematics]: Acceleration" + kinematics.getLinearAcceleration().toString()); // Acceleration
    }

    private void GoTarget(List<Integer> ActiveTargets){
        int index = ActiveTargets.size();
        int i = 0;
        double[] distance = new double[2];

        Log.i(TAG,"ActiveTarget"+ActiveTargets.toString());
        // change the order of the goal targets to be the shortest distance
        if (index == 2) {
            distance[0] = minimum_distance(Global.Nowplace,ActiveTargets.get(0)-1);
            distance[1] = minimum_distance(Global.Nowplace,ActiveTargets.get(1)-1);
            if(distance[0] > distance[1]){
                // swap order
                int temp = ActiveTargets.get(0);
                ActiveTargets.set(0,ActiveTargets.get(1));
                ActiveTargets.set(1,temp);
                Log.i(TAG,"Exchange Active Targets"+ActiveTargets.toString());
            }
        }
        //

        while(i < index) {
            Log.i(TAG, "Let's go Target" + ActiveTargets.get(i).toString());
            Log.i(TAG,"Current position in Gotarget"+Global.Nowplace);
            List<Integer>route = dijkstra(Global.Nowplace,ActiveTargets.get(i)-1); //-1 modifies to zero origin
            Log.i(TAG,"Route"+route.toString());

            for(int n = 1; n<route.size();n++){ //skip n = 0 as it is the starting point
                //Log.i(TAG, "Let's go to node " +route.get(n).toString());
                Waypoint2Number(route.get(n));
            }
            api.laserControl(true);
            api.takeTargetSnapshot(ActiveTargets.get(i));
            ++i;
        }
    }

    public static List<Integer> dijkstra(int start, int end) {
        double[][] A = adjacency_matrix.graph;
        int n = A.length; // number of vertices
        double[] distances = new double[n]; // shortest distance from start point to each vertex
        boolean[] visited = new boolean[n]; // vertex visited state
        int[] prev = new int[n]; //previous vertex
        List<Integer> path = new ArrayList<>(); // save path

        // Initialize the distances array and set vertices other than the starting point to infinity
        Arrays. fill(distances, INF);
        distances[start] = 0.0;

        for (int i = 0; i < n; i++) {
            // Find the unvisited vertex with the smallest distance
            double minDist = INF;
            int minIndex = -1;

            for (int j = 0; j < n; j++) {
                if (!visited[j] && distances[j] < minDist) {
                    minDist = distances[j];
                    minIndex = j;
                }
            }

            // exit if not found
            if (minIndex == -1) {
                break;
            }

            // marks the found vertex as visited
            visited[minIndex] = true;

            // update the distance of neighboring vertices
            for (int j = 0; j < n; j++) {
                if (!visited[j] && A[minIndex][j] != INF) {
                    double distance = distances[minIndex] + A[minIndex][j];
                    if (distance < distances[j]) {
                        distances[j] = distance;
                        prev[j] = minIndex;
                    }
                }
            }
        }
        if (distances[end] == INF) {
            return path; // returns an empty list if unreachable
        }
        // restore shortest path from end point to start point
        int current = end;
        path.add(current);
        while (current != start) {
            current = prev[current];
            path.add(0,current);
        }
        path.add(0,start);
        return path;
    }
    // Waypoint number when considered with zero origin
    private void Waypoint2Number(int n){
        Global.Nowplace = n; // change current position
        Log.i(TAG,"Now_place is "+ Global.Nowplace);
        switch (n) {
            case 0:
                MoveToWaypoint(waypoints_config.point1);
                break;
            case 1:
                MoveToWaypoint(waypoints_config.point2);
                break;
            case 2:
                MoveToWaypoint(waypoints_config.point3);
                break;
            case 3:
                MoveToWaypoint(waypoints_config.point4);
                break;
            case 4:
                MoveToWaypoint(waypoints_config.point5);
                break;
            case 5:
                MoveToWaypoint(waypoints_config.point6);
                break;
            case 6:
                MoveToWaypoint(waypoints_config.goal_point);
                break;
            case 7:
                MoveToWaypoint(waypoints_config.wp1);
                break;
            case 8:
                MoveToWaypoint(waypoints_config.wp2);
                break;
            case 9:
                MoveToWaypoint(waypoints_config.wp3);
                break;
        }
    }

    /**
     * FUNCTIONs ABOUT QRCODE
     */
    private static final String TAGS = "QRCodeReader"; // Replace with your actual TAG


    private static final String QR_CODE_JEM = "JEM";
    private static final String QR_CODE_COLUMBUS = "COLUMBUS";
    private static final String QR_CODE_RACK1 = "RACK1";
    private static final String QR_CODE_ASTROBEE = "ASTROBEE";
    private static final String QR_CODE_INTBALL = "INTBALL";
    private static final String QR_CODE_BLANK = "BLANK";

    private static final String MESSAGE_STAY_AT_JEM = "STAY_AT_JEM";
    private static final String MESSAGE_GO_TO_COLUMBUS = "GO_TO_COLUMBUS";
    private static final String MESSAGE_CHECK_RACK_1 = "CHECK_RACK_1";
    private static final String MESSAGE_I_AM_HERE = "I_AM_HERE";
    private static final String MESSAGE_LOOKING_FORWARD_TO_SEE_YOU = "LOOKING_FORWARD_TO_SEE_YOU";
    private static final String MESSAGE_NO_PROBLEM = "NO_PROBLEM";

    private String readQRCode(Mat image) {
        String QRCodeContent = "";
        try {
            Mat miniImage = new Mat(image, new Rect(700, 360, 240, 240));
            MatOfPoint2f points = new MatOfPoint2f();
            Mat straightQRCode = new Mat();
            QRCodeDetector qrCodeDetector = new QRCodeDetector();

            boolean detectSuccess = qrCodeDetector.detect(miniImage, points);
            Log.i(TAGS, "Detection success is " + detectSuccess);

            QRCodeContent = qrCodeDetector.detectAndDecode(miniImage, points, straightQRCode);
            Log.i(TAGS, "QRCode content is " + QRCodeContent);

            if (QRCodeContent != null) {
                Mat straightQRCodeGray = new Mat();
                straightQRCode.convertTo(straightQRCodeGray, CvType.CV_8UC1);
            }

        } catch (Exception e) {
            Log.e(TAGS, "An error occurred while reading the QR code: ", e);
        }

        return translateQRCodeContent(QRCodeContent);
    }

    private String translateQRCodeContent(String QRCodeContent) {
        switch (QRCodeContent) {
            case QR_CODE_JEM:
                return MESSAGE_STAY_AT_JEM;
            case QR_CODE_COLUMBUS:
                return MESSAGE_GO_TO_COLUMBUS;
            case QR_CODE_RACK1:
                return MESSAGE_CHECK_RACK_1;
            case QR_CODE_ASTROBEE:
                return MESSAGE_I_AM_HERE;
            case QR_CODE_INTBALL:
                return MESSAGE_LOOKING_FORWARD_TO_SEE_YOU;
            case QR_CODE_BLANK:
                return MESSAGE_NO_PROBLEM;
            default:
                Log.e(TAGS, "Unrecognized QR code content: " + QRCodeContent);
                return "";
        }
    }


    private double minimum_distance(int start,int end){
        List<Integer> route = dijkstra(start,end);
        double distance = 0;
        for (int n = 0; n < route.size() - 1; n++) {
            distance = adjacency_matrix.graph[route.get(n)][route.get(n + 1)];
        }
        return distance;

    }
}
