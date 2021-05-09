package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.graphics.Bitmap;
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

    Mat QR_snap;
    Mat AR_snap;

    @Override
    protected void runPlan1(){

        String QRPointA = null;
        double QRData[]; // KOZ Pattern // A'X // A'Y // A'Z
        int max_try = 4;
        int try_read = 0;
        int AR_cast = 0;
        int sleep_time = 3;
        double ncOffset_y = 0.042 ,ncOffset_z  = 0.083;

        // Mission Start //
        api.startMission();

        //moveToWrapper(10.5,-9.8,4.6,0,0,-0.707,0.707,0);
        moveToWrapper(11.21+ncOffset_y,-9.6,4.79+ncOffset_z,0,0,-0.707,0.707,5);
        try {
            sleep(sleep_time);
        } catch (Exception e) {
            Log.d("Sleep status", "sleep failed");
        }
        /////////////////////////////////////////////////////////////////////////////

        while(QRPointA == null && try_read < max_try) {
            QR_snap = api.getMatNavCam();

            Bitmap QR_Bitmap = null;
            Mat QR_mat = imProveImaging(QR_snap, false);
            Log.d("IMGPROC", "Bitmap conversion started");
            try {
                QR_Bitmap = Bitmap.createBitmap(QR_mat.cols(), QR_mat.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(QR_mat, QR_Bitmap);
                Log.d("IMGPROC", "Bitmap conversion finished");
                QRPointA = QRscaner(QR_Bitmap);
            }
            catch (CvException e){Log.d("Exception",e.getMessage());}


           // Bitmap QR_crop = Bitmap.createBitmap(QR_snap,0,0,960,960);

            try_read++;
            //////////////////////////////////////////////////////////////////////////////////////
        }
        Log.d("QR", QRPointA);
        api.sendDiscoveredQR(QRPointA);
        QRData = parseQRinfo(QRPointA);
        Log.d("QR", ""+ QRData[0] + ", " + QRData[1] + ", "+ QRData[2] + ", "+ QRData[3]);
        ////////////////////////// AR PROCESS //////////////////////////

        AR_result targetData = AR_scanAndLocalize(false);
        Point target = targetData.getTargetLocation();


        Kinematics astrobee = api.getTrustedRobotKinematics();
        Point point = astrobee.getPosition();
        Quaternion IgniteAngle = rotationCalculator(point.getX(), point.getY(), point.getZ()+0.1111, target.getX(), -10.25, target.getZ());
        Log.d("AR_RESULTa", "" +  IgniteAngle.getX() + " "+ IgniteAngle.getY() + " "+IgniteAngle.getZ() + " "+ IgniteAngle.getW() + " ");
        moveToWrapper(point.getX(), point.getY(), point.getZ(), IgniteAngle.getX(), IgniteAngle.getY(), IgniteAngle.getZ(), IgniteAngle.getW(), 2);


        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);


        moveToWrapper(10.5,-8.9,4.5,0,0,-0.707,0.707,0);
        moveToWrapper(10.6,-8,4.5,0,0,-0.707,0.707,0);

        api.reportMissionCompletion();

    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    class AR_result  {

        private double[] AR_ids = new double[4];
        private double[][] tvecs = new double[4][3];
        private double selected_id;

        public AR_result (Mat ids, Mat _tvecs) {
            Log.d("AR_RESULT", "Class called");
            Log.d("AR_RESULT", ids.dump());
            Log.d("AR_RESULT", ""+ids.rows());
//            this.AR_ids = ids;_
//            this.tvecs = _tvecs;
            if (ids.rows() == 4) { // get all ar's corner
                this.selected_id = 0.0; // means we got full package
                ///////// sorting ////////
                Log.d("AR_RESULT", "Start sort");
                for (int i = 0;i < 4;i++) {
                    Log.d("AR_RESULT", "sorting "+(int)(ids.get(i,0)[0]));
                    if ((int)(ids.get(i,0)[0]) == 1) {
                        this.AR_ids[0] = ids.get(i,0)[0];
                        this.tvecs[0][0] = _tvecs.get(i,0)[0];
                        this.tvecs[0][1] = _tvecs.get(i,0)[1];
                        this.tvecs[0][2] = _tvecs.get(i,0)[2];
                    } else if ((int)(ids.get(i,0)[0]) == 2) {
                        this.AR_ids[1] = ids.get(i,0)[0];
                        this.tvecs[1][0] = _tvecs.get(i,0)[0];
                        this.tvecs[1][1] = _tvecs.get(i,0)[1];
                        this.tvecs[1][2] = _tvecs.get(i,0)[2];
                    } else if ((int)(ids.get(i,0)[0]) == 3) {
                        this.AR_ids[2] = ids.get(i,0)[0];
                        this.tvecs[2][0] = _tvecs.get(i,0)[0];
                        this.tvecs[2][1] = _tvecs.get(i,0)[1];
                        this.tvecs[2][2] = _tvecs.get(i,0)[2];
                    }else if ((int)(ids.get(i,0)[0]) == 4) {
                        this.AR_ids[3] = ids.get(i,0)[0];
                        this.tvecs[3][0] = _tvecs.get(i,0)[0];
                        this.tvecs[3][1] = _tvecs.get(i,0)[1];
                        this.tvecs[3][2] = _tvecs.get(i,0)[2];
                    }
                }
//                Log.d("AR_RESULTa", "" + ids.get(0,0)[0] + " "+ ids.get(1,0)[0] + " "+ ids.get(2,0)[0] + " "+ ids.get(3,0)[0] + " ");
//                Log.d("AR_RESULTa", "" + _tvecs.get(0,0)[0] + " "+  _tvecs.get(0,0)[1] + " "+ _tvecs.get(0,0)[2]);
//                Log.d("AR_RESULTa", "" + _tvecs.get(1,0)[0] + " "+  _tvecs.get(1,0)[1] + " "+ _tvecs.get(1,0)[2]);
//                Log.d("AR_RESULTa", "" + _tvecs.get(2,0)[0] + " "+  _tvecs.get(2,0)[1] + " "+ _tvecs.get(2,0)[2]);
//                Log.d("AR_RESULTa", "" + _tvecs.get(3,0)[0] + " "+  _tvecs.get(3,0)[1] + " "+ _tvecs.get(3,0)[2]);
                Log.d("AR_RESULT", "End sort");
                Log.d("AR_RESULT", Arrays.toString(AR_ids));
                Log.d("AR_RESULT",  Arrays.toString(tvecs[0]));
                Log.d("AR_RESULT",  Arrays.toString(tvecs[1]));
                Log.d("AR_RESULT",  Arrays.toString(tvecs[2]));
                Log.d("AR_RESULT",  Arrays.toString(tvecs[3]));
            } else if (ids.rows() != 0 && ids.rows() < 4) { // get first ar and let loose
                this.selected_id = ids.get(0,0)[0];
                this.tvecs[0][0] = _tvecs.get(0,0)[0];
                this.tvecs[0][1] = _tvecs.get(0,0)[1];
                this.tvecs[0][2] = _tvecs.get(0,0)[2];
            } else {
                Log.e("AR discover", "Failed To read Array");
            }
        }

        public Point getTargetLocation () {
            double center_w = 0.0;
            double center_h = 0.0;
            double depth_y = 0.0;
            double[] result = new double[3];

            if (selected_id == 0.0) { // all 4 ar
                center_w = (((tvecs[0][0] + tvecs[1][0]) / 2) + ((tvecs[2][0] + tvecs[3][0]) / 2)) / 2;
                center_h = (((tvecs[0][1] + tvecs[3][1]) / 2) + ((tvecs[1][1] + tvecs[2][1]) / 2)) / 2;
                depth_y = (tvecs[0][2] + tvecs[1][2] + tvecs[2][2] + tvecs[3][2]) / 4;
            }
            Log.d("calculateDat", "W: "+ center_w + " H: "+ center_h + " depth: " + depth_y);
            Kinematics astrobee = api.getTrustedRobotKinematics();
            Point _point = astrobee.getPosition();
            result[0] = _point.getX() + center_w;
            result[1] = _point.getY() - depth_y;
            result[2] = _point.getZ() + center_h;
            Log.d("TargetPos", "X: "+ result[0] + " Y: "+ result[1] + " Z: " + result[2]);

            return new Point(result[0],result[1],result[2]);
        }


    }






    public AR_result AR_scanAndLocalize(boolean status) {
        int row = 0, col = 0;
        double[][] NavCamInst = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
//        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        cameraMatrix.put(row, col, NavCamInst[0]);
        distCoeffs.put(row, col, NavCamInst[1]);
        Log.d("cameraMatrix", cameraMatrix.dump());
        Log.d("distCoeffs", distCoeffs.dump());

        Mat ids = new Mat();
        Mat tvecs = new Mat();
        Mat rvecs = new Mat();
        List<Mat> corners = new ArrayList<>();
        List<Mat> rej = new ArrayList<>();
        double result = 0.0;
        Log.d("AR", "Started reading AR");
        while (result == 0) {
            AR_snap = api.getMatNavCam();
            Mat AR_mat = imProveImaging(AR_snap, false);
            try {
                DetectorParameters parameters = DetectorParameters.create();
//                parameters.set_errorCorrectionRate(1);
                if (status) {
                    parameters.set_adaptiveThreshWinSizeMin(5);
                    parameters.set_adaptiveThreshWinSizeMax(29);
                    parameters.set_adaptiveThreshWinSizeStep(4);
                }
                Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
                Aruco.detectMarkers(AR_mat, dictionary, corners, ids, parameters,rej,cameraMatrix,distCoeffs);
                if (!ids.empty()) {
                    Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);
                    result = ids.get(0, 0)[0];
                    Log.d("AR_IDs", "Result : " + result);
                    Log.d("AR_IDs", ids.dump());
                    Log.d("AR_IDs", "col "+ids.cols()+" row "+ids.rows());
                    Log.d("AR_IDs", "col "+tvecs.cols()+" row "+tvecs.rows()+ " len "+tvecs.get(0,0).length);
                    break;
                }
            } catch (Exception e) {
                Log.d("AR discover:", "Not detected");
                e.printStackTrace();
            }
        }
        return new AR_result(ids, tvecs);
    }

    public double[] parseQRinfo(String QRData)
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

    public String QRscaner(Bitmap source) // removed static //
    {
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
        return result;
    }


    public  Mat imProveImaging(Mat source, boolean undistort_state) {
        int row = 0, col = 0;
        double[][] NavCamInst = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        cameraMatrix.put(row, col, NavCamInst[0]);
        distCoeffs.put(row, col, NavCamInst[1]);
        Log.d("cameraMatrix", cameraMatrix.dump());
        Log.d("distCoeffs", distCoeffs.dump());
        Mat Thres = new Mat();
        Mat result = new Mat();
        if (undistort_state) {
            Log.d("IMGPROC", "Undistortion process started");
            Imgproc.undistort(source, dst, cameraMatrix, distCoeffs);
            Log.d("IMGPROC", "Undistortion process finished");
        } else {
            Log.d("IMGPROC", "Skipping Undistortion process");
            dst = source;
        }

        Log.d("IMGPROC", "Threshold process started");
        Imgproc.threshold(dst, Thres, 240, 255, Imgproc.THRESH_TOZERO);
        Log.d("IMGPROC", "Threshold process finished");
        Log.d("IMGPROC", "Contour location process started");
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<MatOfPoint> Wanted_contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(Thres, contours, new Mat(), Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
        for(int i=0; i< contours.size();i++)
        {
            if ((Imgproc.contourArea(contours.get(i)) >7000))//Imgproc.contourArea(contours.get(i)) > 50
            {
                Wanted_contours.add(contours.get(i));
                System.out.println(contours.get(i).dump());
            }
        }
        Log.d("IMGPROC", "Contour location process finished");
        Log.d("IMGPROC", "Filling process started");
        Mat mask = new Mat( new Size( dst.cols(), dst.rows() ), CvType.CV_8UC1 );
        Imgproc.drawContours(dst, Wanted_contours, -1, new Scalar(255,255,255), 3);
        Imgproc.fillPoly(mask , Wanted_contours, new Scalar(255, 255, 255));
        Core.bitwise_and(dst,mask,result);
        Log.d("IMGPROC", "Filling process finished");

        return result;
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

