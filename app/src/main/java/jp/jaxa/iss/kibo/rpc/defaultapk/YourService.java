package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import sensor_msgs.PointCloud2;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;


import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;



/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    Mat airlock_snap; // will be initialize upon reading QR
    double ncOffset_x = 0.042 , ncOffset_y = 0.117, ncOffset_z  = 0.083;
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
    Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

    @Override
    protected void runPlan1(){

        String QRPointA = null;
        int max_try = 4;
        int try_read = 0;
        int AR_cast = 0;
        int sleep_time = 3;

        // Initialize camera intrinsic values //
        double[][] NavCamInst = api.getNavCamIntrinsics();
        cameraMatrix.put(0, 0, NavCamInst[0]);
        distCoeffs.put(0, 0, NavCamInst[1]);
        Log.d("cameraMatrix", cameraMatrix.dump());
        Log.d("distCoeffs", distCoeffs.dump());

        // Mission Start //
        api.startMission();

        //moveToWrapper(10.5,-9.8,4.6,0,0,-0.707,0.707,0);
        moveToWrapper(11.21+ncOffset_x,-9.6+ncOffset_y,4.79+ncOffset_z,0,0,-0.707,0.707,5);
        try {
            sleep(sleep_time);
        } catch (Exception e) {
            Log.d("Sleep status", "sleep failed");
        }
        QRPointA = QR_method(max_try);
        api.sendDiscoveredQR(QRPointA);
        
//        QRData = parseQRinfo(QRPointA);
//        Log.d("QR", ""+ QRData[0] + ", " + QRData[1] + ", "+ QRData[2] + ", "+ QRData[3]);


        ////////////////////////// AR PROCESS //////////////////////////

//        AR_result targetData = AR_scanAndLocalize(false);
//        Point target = targetData.getTargetLocation();
//
//
////        Kinematics astrobee = api.getTrustedRobotKinematics();
////        Point point = astrobee.getPosition();
//        Quaternion IgniteAngle = rotationCalculator(11.21+ncOffset_x,-9.6+ncOffset_y,4.79+ncOffset_z, target.getX()-0.057+0.035, target.getY()+0.1302, target.getZ()+0.2111);
//        Log.d("AR_RESULTa", "" +  IgniteAngle.getX() + " "+ IgniteAngle.getY() + " "+IgniteAngle.getZ() + " "+ IgniteAngle.getW() + " ");
//        moveToWrapper(11.21+ncOffset_x,-9.6+ncOffset_y,4.79+ncOffset_z, IgniteAngle.getX(), IgniteAngle.getY(), IgniteAngle.getZ(), IgniteAngle.getW(), 2);
//
//
//        api.laserControl(true);
//        api.takeSnapshot();
//        api.laserControl(false);
//
//
//        moveToWrapper(10.5,-8.9,4.5,0,0,-0.707,0.707,0);
//        moveToWrapper(10.6,-8,4.5,0,0,-0.707,0.707,0);
//
//
//
//        api.reportMissionCompletion();

    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }



//    public AR_result AR_scanAndLocalize(boolean status) {
//        long start_time = SystemClock.elapsedRealtime();
//        int row = 0, col = 0;
//        double[][] NavCamInst = api.getNavCamIntrinsics();
//        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
//        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
////        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
//        cameraMatrix.put(row, col, NavCamInst[0]);
//        distCoeffs.put(row, col, NavCamInst[1]);
//        Log.d("cameraMatrix", cameraMatrix.dump());
//        Log.d("distCoeffs", distCoeffs.dump());
//
//        Mat ids = new Mat();
//        Mat tvecs = new Mat();
//        Mat rvecs = new Mat();
//        List<Mat> corners = new ArrayList<>();
//        List<Mat> rej = new ArrayList<>();
//        double result = 0.0;
//        Log.d("AR", "Started reading AR");
//        while (result == 0) {
//            AR_snap = api.getMatNavCam();
//            //Mat AR_mat = imProveImaging(AR_snap, false, 2);
//            try {
//                DetectorParameters parameters = DetectorParameters.create();
////                parameters.set_errorCorrectionRate(1);
//                if (status) {
//                    parameters.set_adaptiveThreshWinSizeMin(5);
//                    parameters.set_adaptiveThreshWinSizeMax(29);
//                    parameters.set_adaptiveThreshWinSizeStep(4);
//                }
//                Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
//                Aruco.detectMarkers(AR_snap, dictionary, corners, ids, parameters,rej,cameraMatrix,distCoeffs);
//                if (!ids.empty()) {
//                    Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);
//                    result = ids.get(0, 0)[0];
//                    Log.d("AR_IDs", "Result : " + result);
//                    Log.d("AR_IDs", ids.dump());
//                    Log.d("AR_IDs", "col "+ids.cols()+" row "+ids.rows());
//                    Log.d("AR_IDs", "col "+tvecs.cols()+" row "+tvecs.rows()+ " len "+tvecs.get(0,0).length);
//                    break;
//                }
//            } catch (Exception e) {
//                Log.d("AR discover:", "Not detected");
//                e.printStackTrace();
//            }
//        }
//        long stop_time = SystemClock.elapsedRealtime();
//        Log.d("AR_TIME:"," "+ (stop_time-start_time)/1000);
//        return new AR_result(ids, tvecs);
//    }

    public double[] parseQRinfo(String QRData) // no need
    {
        String[] remnants = QRData.split(",");

        double parsedDouble[] = new double[4];
        for (int i = 0;i< remnants.length;i++) {
            parsedDouble[i] = Double.parseDouble(remnants[i].replaceAll("[^0-9. ]", ""));
        }

        return parsedDouble;
    }

    public void sleep(int timer) throws InterruptedException
    {
        TimeUnit.SECONDS.sleep(timer);
    }

    public String QR_method(int maxRetryTimes) {
        String QRPointA = null;
        int retryTimes = 0;
        while(QRPointA == null && retryTimes < maxRetryTimes) {
            airlock_snap = api.getMatNavCam();

            Bitmap QR_Bitmap;
            Mat QR_mat = QRFocus(airlock_snap);
            Log.d("IMGPROC", "Bitmap conversion started");
            try {
                QR_Bitmap = Bitmap.createBitmap(QR_mat.cols(), QR_mat.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(QR_mat, QR_Bitmap);
                Log.d("IMGPROC", "Bitmap conversion finished");
                QRPointA = QRscaner(QR_Bitmap);
            }
            catch (CvException e){Log.d("Exception",e.getMessage());}
            // Bitmap QR_crop = Bitmap.createBitmap(QR_snap,0,0,960,960);
            retryTimes++;
        }

        if (QRPointA == null) { // if the filter doesn't work --> process through the entire bitmap
            retryTimes = 0;
            while(QRPointA == null && retryTimes < maxRetryTimes) {
                Bitmap QR_revive = api.getBitmapNavCam();
                QRPointA = QRscaner(QR_revive);
                retryTimes++;
            }
        }
        Log.d("QR", QRPointA);
        return QRPointA;
    }

    public String QRscaner(Bitmap source) // removed static //
    {
        long start_time = SystemClock.elapsedRealtime();
        String result = null;

        int[] pixel = new int[source.getWidth()*source.getHeight()];
        source.getPixels(pixel, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());
        Image barcode = new Image(source.getWidth() ,source.getHeight() ,"RGB4");
        barcode.setData(pixel);
        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
        reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
        reader.scanImage(barcode.convert("Y800"));

        try
        {
            SymbolSet syms = reader.getResults();
            for(Symbol sym : syms)
            {
                result = sym.getData();
            }
        }
        catch(Exception e)
        {
            Log.d("QR discover: ","Not detected");
        }
        long stop_time = SystemClock.elapsedRealtime();
        Log.d("QR_TIME:"," "+ (stop_time-start_time)/1000);
        return result;
    }


    public  Mat QRFocus(Mat source) { // 1 qr 2 ar 3 etc
        long start_time = SystemClock.elapsedRealtime();

        Mat thres = new Mat();
        Mat result = new Mat();
        Log.d("IMGPROC", "Threshold process started");
        Imgproc.threshold(source, thres, 240, 255, Imgproc.THRESH_TOZERO);
        Log.d("IMGPROC", "Threshold process finished");
        Log.d("IMGPROC", "Contour location process started");
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<MatOfPoint> Wanted_contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(thres, contours, new Mat(), Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

        Rect boundingBox = new Rect();

        for(int i=0; i< contours.size();i++)
        {
            if ((Imgproc.contourArea(contours.get(i)) >7000))//Imgproc.contourArea(contours.get(i)) > 50
            {
                boundingBox = Imgproc.boundingRect(contours.get(i));
                if (boundingBox.height > boundingBox.width) {
                    Wanted_contours.add(contours.get(i));
                   // System.out.println(contours.get(i).dump());
                }
            }
        }
        Log.d("IMGPROC", "Contour location process finished");
        Log.d("IMGPROC", "Filling process started");
        Mat mask = new Mat( new Size( source.cols(), source.rows() ), CvType.CV_8UC1 );
        Imgproc.drawContours(source, Wanted_contours, -1, new Scalar(255,255,255), 3);
        Imgproc.fillPoly(mask , Wanted_contours, new Scalar(255, 255, 255));
        Core.bitwise_and(source,mask,result);
        Log.d("IMGPROC", "Filling process finished");

        long stop_time = SystemClock.elapsedRealtime();
        Log.d("IMGPROC_TIME:"," "+ (stop_time-start_time)/1000);

        return result.submat(boundingBox.y, boundingBox.y + boundingBox.height, boundingBox.x, boundingBox.x + boundingBox.width);


    }


    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w, final int LOOP_MAX)
    {
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }


    private Quaternion rotationCalculator(double x, double y, double z, double xp, double yp,double zp)
    {


        double x_sub,y_sub,z_sub,x_deg,z_deg;
        x_sub  = xp-x; // + is right // - is left
        y_sub = Math.abs(yp-y); // always absolute

        double bftan = x_sub/y_sub;
        z_deg = Math.toDegrees(Math.atan(bftan));
        double z_deg_final = z_deg - 90;

        z_sub = zp-z; // + is down // - is up
        bftan = z_sub/y_sub;
        x_deg = -Math.toDegrees(Math.atan(bftan));

        z_deg = Math.toRadians(z_deg_final);
        x_deg = Math.toRadians(x_deg);
        double cy = Math.cos(z_deg * 0.5);
        double sy = Math.sin(z_deg * 0.5);
        double cp = Math.cos(0 * 0.5);
        double sp = Math.sin(0 * 0.5);
        double cr = Math.cos(x_deg * 0.5);
        double sr = Math.sin(x_deg * 0.5);

        double qua_w = cr * cp * cy + sr * sp * sy;
        double qua_x = sr * cp * cy - cr * sp * sy;
        double qua_y = -(cr * sp * cy + sr * cp * sy);
        double qua_z = cr * cp * sy - sr * sp * cy;
        Quaternion q = new Quaternion((float)qua_x,(float)qua_y,(float)qua_z,(float)qua_w);


        return q;
    }

}

