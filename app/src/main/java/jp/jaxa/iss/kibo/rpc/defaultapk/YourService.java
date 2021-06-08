package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
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
import java.util.HashMap;
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
    Point gotoPos;
    Mat cameraMatrix;
    Mat distCoeffs;


    
    @Override
    protected void runPlan1(){
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
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
        gotoPos = new Point(11.27, -9.5, 4.79); // edit here
        moveToWrapper(gotoPos.getX(),gotoPos.getY(),gotoPos.getZ(),0,0,-0.707,0.707,5);
        try {
            sleep(sleep_time);
        } catch (Exception e) {
            Log.d("Sleep status", "sleep failed");
        }
        QRPointA = QR_method(max_try);
        api.sendDiscoveredQR(QRPointA);

//        QRData = parseQRinfo(QRPointA);
//        Log.d("QR", ""+ QRData[0] + ", " + QRData[1] + ", "+ QRData[2] + ", "+ QRData[3]);

        ARProcessing ARbee = new ARProcessing(airlock_snap);
        Point target = ARbee.getTargetPosition();
        ////////////////////////// AR PROCESS //////////////////////////

        // shooting process //
//        double[] parkingPos = new double[3];
//        parkingPos[0] = target.getX() - 0.0572;
//        parkingPos[1] = gotoPos.getY(); // current Y
//        parkingPos[2] = gotoPos.getZ(); // current Z
        myMathmanager celes = new myMathmanager();
        Quaternion IgniteAngle = celes.rotationCalculator(gotoPos.getX()+0.0572+0.115,gotoPos.getY(),gotoPos.getZ()-0.1111-0.075, target.getX(), target.getY(), target.getZ(), 0);
        moveToWrapper(gotoPos.getX(),gotoPos.getY(),gotoPos.getZ(), IgniteAngle.getX(), IgniteAngle.getY(), IgniteAngle.getZ(), IgniteAngle.getW(), 2);
        //////////////////////

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

    class ARProcessing {

        private Mat ids;
        private ArrayList<Mat> corners;
        private Mat rvecs;
        private Mat tvecs;
        private Mat image;
        private HashMap<Integer, Mat> ARRotations;
        private HashMap<Integer, Mat> ARTranslation;
        private HashMap<Integer, double[]> AR_LookUpTable; // might try sparseIntArray later on for lesser memory allocation
        private myMathmanager demon;

        public ARProcessing(Mat image) {
            this.image = image;
            this.rvecs = new Mat();
            this.tvecs = new Mat();
            this.ARRotations = new HashMap<Integer, Mat>();
            this.ARTranslation = new HashMap<Integer, Mat>();
            this.AR_LookUpTable = new HashMap<Integer, double[]>();
            // Define AR Look up Table for each id //
            this.AR_LookUpTable.put(1, new double[] {-0.1125, 0.415}); // -x // +z
            this.AR_LookUpTable.put(2, new double[] {0.1125, 0.415}); // +x // +z
            this.AR_LookUpTable.put(3, new double[] {0.1125, -0.415}); // +x // -z
            this.AR_LookUpTable.put(4, new double[] {-0.1125, -0.415}); // -x // -z
            demon = new myMathmanager();
        }

        private void ARDetect() {
            long start_time = SystemClock.elapsedRealtime();
            corners = new ArrayList<>();
            ids = new Mat();

            List<Mat> rej = new ArrayList<>();
            double result = 0.0;
            Log.d("AR", "Started reading AR");
            while (result == 0) {
                image = api.getMatNavCam();
                try {
                    DetectorParameters parameters = DetectorParameters.create();
                    Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
                    Aruco.detectMarkers(image, dictionary, corners, ids, parameters,rej,cameraMatrix,distCoeffs);
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
            long stop_time = SystemClock.elapsedRealtime();
            Log.d("AR_TIME:"," "+ (stop_time-start_time)/1000);
        }

        private int[] getTrustedTargetPlane() {
            this.ARDetect();
            if (tvecs == null || rvecs == null) {
                Log.d("ARProcessing","Either tvecs or rvecs is null");
                return new int[] {0,0};
            }
            HashMap<Integer, double[]> ARCandidates = new HashMap<Integer, double[]>();

            for(int i=0;i<rvecs.size().height;i++) {
                Mat rvec=new Mat(1,3,CvType.CV_64FC1);
                Mat tvec=new Mat(1,3,CvType.CV_64FC1);
                Mat rot=new Mat();
                rvec.put(0, 0, rvecs.get(i,0));
                tvec.put(0, 0, tvecs.get(i,0));
                Calib3d.Rodrigues(rvec, rot);
                // Put in hashmap for easy iteration next loop
                ARRotations.put((int)ids.get(i, 0)[0], rot);
                ARTranslation.put((int)ids.get(i, 0)[0], tvec);
                ARCandidates.put((int)ids.get(i, 0)[0], demon.ToEuler(demon.rotMatToQuaternions(rot)));

                Log.d("mathmanager",(int)ids.get(i, 0)[0] + " : " +Arrays.toString( demon.ToEuler(demon.rotMatToQuaternions(rot))));
            }

            // x y z sorting
            double[] diff = new double[4];
            for (int i=0;i<3;i++) { // final index is its id
                double mean = 0;
                int index = -999;

                for (int j=1;j <= 4;j++) {
                    if (ARCandidates.get(j) == null) continue;
                    mean += ARCandidates.get(j)[i];
                }
                mean /= ARCandidates.size();


                for (int j = 1;j <= 4;j++) {
                    if (ARCandidates.get(j) == null) {
                        diff[j-1] = -999;
                        continue;
                    }
                    diff[j-1] = Math.abs(mean - ARCandidates.get(j)[i]);
                }
                double maxValue = diff[0];
                if (diff[0] > diff[1] && diff[0] > diff[2] && diff[0] > diff[3]) index = 1;
                for (int j = 0;j < 4;j++) {
                    if(diff[j] > maxValue){
                        maxValue = diff[j];
                        index = j+1;
                    }

                }

                if (ARCandidates.get(index) != null) ARCandidates.remove(index);

            }

            if (ARCandidates.size() == 4) {
                Log.d("ARProcessing","ALL Markers are usable, Preparing to average the tvecs and rvecs . . .");
                return new int[] {1, 2, 3, 4};
            } else {
                Log.d("ARProcessing","Not all 4 Marker are usable, selecting only 1 tvecs and rvecs from the first index . . .");
            }


            for (int j = 1;j< 5 ;j++) {
                if (ARCandidates.get(j) == null) continue;
                Log.d("ARProcessing",j + " >> " +"roll: " + ARCandidates.get(j)[0] + " pitch: " +  ARCandidates.get(j)[1] + " yaw: " + ARCandidates.get(j)[2]);
                return new int[] {j};
            }


            return null; // no ar found // error

        }

        public Point getTargetPosition() {

            double[] pos = {0 , 0 , 0};
            int[] AR_Ids = this.getTrustedTargetPlane();
            if (rvecs.size().height == 4) { //DEBUG

                for (int i = 0;i < 3;i++) {
                    for (int j = 0;j < 4;j++) {
                        pos[i] += tvecs.get(j,0)[i];
                    }
                    pos[i] /= 4;
                }
                Log.d("ARProcessing",tvecs.dump());
                Log.d("ARProcessing",Arrays.toString(pos));
                double temp = pos[1];
                pos[1] = -pos[2];
                pos[2] = temp;
                Log.d("ARProcessing","All 4 AR is found, ignoring the rotation matrices ... ");
                Log.d("ARProcessing",Arrays.toString(pos));

            } else {

                if (AR_Ids == null) return null;

                Log.d("ARProcessing","AR_IDs: "+AR_Ids[0]);
                double[][] matrix1 = new double[4][4];
                double[] matrix2 = new double[4];
                // AR Frame to Camera Frame //
                matrix1[0][0] = ARRotations.get(AR_Ids[0]).get(0, 0)[0]; matrix1[0][1] = ARRotations.get(AR_Ids[0]).get(0, 1)[0]; matrix1[0][2] = ARRotations.get(AR_Ids[0]).get(0, 2)[0]; matrix1[0][3] = ARTranslation.get(AR_Ids[0]).get(0, 0)[0];
                matrix1[1][0] = ARRotations.get(AR_Ids[0]).get(1, 0)[0]; matrix1[1][1] = ARRotations.get(AR_Ids[0]).get(1, 1)[0]; matrix1[1][2] = ARRotations.get(AR_Ids[0]).get(1, 2)[0]; matrix1[1][3] = ARTranslation.get(AR_Ids[0]).get(0, 1)[0];
                matrix1[2][0] = ARRotations.get(AR_Ids[0]).get(2, 0)[0]; matrix1[2][1] = ARRotations.get(AR_Ids[0]).get(2, 1)[0]; matrix1[2][2] = ARRotations.get(AR_Ids[0]).get(2, 2)[0]; matrix1[2][3] = ARTranslation.get(AR_Ids[0]).get(0, 2)[0];
                matrix1[3][0] = 0; matrix1[3][1] = 0; matrix1[3][2] = 0; matrix1[3][3] = 1;

                matrix2[0] = AR_LookUpTable.get(AR_Ids[0])[0];
                matrix2[1] = 0;
                matrix2[2] = AR_LookUpTable.get(AR_Ids[0])[1];
                matrix2[3] = 1;


                Log.d("ARProcessing","Original tvec" + ARTranslation.get(AR_Ids[0]).dump());
                double[] NC_coords = demon.homogeneousTransform(matrix1, matrix2); // w d h
                Log.d("ARProcessing","New tvec" + Arrays.toString(NC_coords));

                pos[0] = NC_coords[0];
                pos[2] = NC_coords[1];
            }

            // Camera frame to Laser Frame //
//            pos[0] = NC_coords[0] - 0.0994;
//            pos[1] = 0; // -NC_coords[2] + 0.0125
//            pos[2] = NC_coords[1] + 0.0285;
//            // Laser frame to Robot Frame //
//            pos[0] = pos[0] + 0.0572;
//            pos[1] = 0;
//            pos[2] = pos[2] - 0.1111;
           // Kinematics astrobee = api.getTrustedRobotKinematics();
            //   moveToWrapper(10.9,-9.8,4.79,0,0,-0.707,0.707,5);
//            Point point = new Point(10.9,-9.5,4.79);
            // Camera frame to robot frame //
            pos[0] -= 0.0422;
            pos[1] = 0; // -NC_coords[2] + 0.0125
            pos[2] -= 0.0826;
            // Robot frame to Global Frame //
            pos[0] +=  gotoPos.getX();
            pos[1] = -10.585; // inverse transform // 0.1302
            pos[2] += gotoPos.getZ();
            Log.d("ARProcessing",Arrays.toString(pos));
            //System.out.println(demon.rotationCalculator(11.247, -9.483, 4.868, pos[0], pos[1], pos[2]));
            return new Point(pos[0], pos[1], pos[2]);
        }
    }



    class myMathmanager {

        public Quaternion rotMatToQuaternions(Mat rot) {
            float qua_x = 0,qua_y = 0,qua_z = 0,qua_w = 0;
            float tr = (float)rot.get(0, 0)[0] + (float)rot.get(1, 1)[0] + (float)rot.get(2,2)[0]; // m00 + m11 + m22
            if (tr > 0) {
                float S = (float)Math.sqrt(tr+1.0f) * 2; // S=4*qw
                qua_w = 0.25f * S;
                qua_x = ((float)rot.get(2, 1)[0] - (float)rot.get(1, 2)[0]) / S; // (m21 - m12) / S
                qua_y = ((float)rot.get(0, 2)[0] - (float)rot.get(2, 0)[0]) / S; // (m02 - m20) / S
                qua_z = ((float)rot.get(1, 0)[0] - (float)rot.get(0, 1)[0]) / S; // (m10 - m01) / S
            } else if (((float)rot.get(0, 0)[0] > (float)rot.get(1, 1)[0])&((float)rot.get(0, 0)[0] > (float)rot.get(2, 2)[0])) { // (m00 > m11)&(m00 > m22)
                float S = (float)Math.sqrt(1.0f + (float)rot.get(0, 0)[0] - (float)rot.get(1, 1)[0] - (float)rot.get(2, 2)[0]) * 2; // S=4*qx sqrt(1.0 + m00 - m11 - m22) * 2
                qua_w = ((float)rot.get(2, 1)[0] - (float)rot.get(1, 2)[0]) / S; // (m21 - m12) / S
                qua_x = 0.25f * S;
                qua_y = ((float)rot.get(0, 1)[0] + (float)rot.get(1, 0)[0]) / S; // (m01 + m10) / S
                qua_z = ((float)rot.get(0, 2)[0] + (float)rot.get(2, 0)[0]) / S; // (m02 + m20) / S
            } else if ((float)rot.get(1, 1)[0] > (float)rot.get(2, 2)[0]) { // m11 > m22
                float S = (float)Math.sqrt(1.0 + (float)rot.get(1, 1)[0] - (float)rot.get(0, 0)[0] - (float)rot.get(2, 2)[0]) * 2; // S=4*qy sqrt(1.0 + m11 - m00 - m22) * 2
                qua_w = ((float)rot.get(0, 2)[0] - (float)rot.get(2, 0)[0]) / S; // (m02 - m20) / S
                qua_x = ((float)rot.get(0, 1)[0] + (float)rot.get(1, 0)[0]) / S; // (m01 + m10) / S
                qua_y = 0.25f * S;
                qua_z = ((float)rot.get(1, 2)[0] + (float)rot.get(2, 1)[0]) / S; // (m12 + m21) / S
            } else {
                float S = (float)Math.sqrt(1.0 + (float)rot.get(2, 2)[0] - (float)rot.get(0, 0)[0] - (float)rot.get(1, 1)[0]) * 2; // S=4*qz sqrt(1.0 + m22 - m00 - m11) * 2
                qua_w = ((float)rot.get(1, 0)[0] - (float)rot.get(0, 1)[0]) / S; // (m10 - m01) / S
                qua_x = ((float)rot.get(0, 2)[0] + (float)rot.get(2, 0)[0]) / S; // (m02 + m20) / S
                qua_y = ((float)rot.get(1, 2)[0] + (float)rot.get(2, 1)[0]) / S; // (m12 + m21) / S
                qua_z = 0.25f * S;
            }
            return new Quaternion(qua_x,qua_y,qua_z,qua_w);
        }

        public Quaternion normalize(Quaternion q) {
            double x = q.getX();
            double y = q.getY();
            double z = q.getZ();
            double w = q.getW();
            double norm = Math.sqrt(x*x + y*y + z*z + w*w);
            double x_,y_,z_,w_;
            x_ = x / norm;
            y_ = y / norm;
            z_ = z / norm;
            w_ = w / norm;

            return new Quaternion((float)x_,(float)y_,(float)z_,(float)w_);
        }

        public double[] ToEuler(Quaternion q) {
            // EULER ZYX ORDER
            double x = q.getX();
            double y = q.getY();
            double z = q.getZ();
            double w = q.getW();
            double heading = 0;
            double attitude = 0;
            double bank = 0;
            double sqw = w*w;
            double sqx = x*x;
            double sqy = y*y;
            double sqz = z*z;
            double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
            double test = x*y + z*w;
            if (test > 0.499*unit) { // singularity at north pole
                heading = Math.toDegrees(2 * Math.atan2(x,w));
                attitude = Math.toDegrees(Math.PI/2);
                bank = 0;
                return new double[] {bank,heading,attitude};
            }
            if (test < -0.499*unit) { // singularity at south pole
                heading = Math.toDegrees(-2 * Math.atan2(x,w));
                attitude = Math.toDegrees(-Math.PI/2);
                bank = 0;
                return new double[] {bank,heading,attitude};
            }
            heading = Math.toDegrees(Math.atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw));
            attitude = Math.toDegrees(Math.asin(2*test/unit));
            bank = Math.toDegrees(Math.atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw));
            return new double[] {bank,heading,attitude};
        }

        public Quaternion ToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
        {
            roll = Math.toRadians(roll);
            pitch = Math.toRadians(pitch);
            yaw = Math.toRadians(yaw);
            // Abbreviations for the various angular functions
            double cy = Math.cos(yaw * 0.5);
            double sy = Math.sin(yaw * 0.5);
            double cp = Math.cos(pitch * 0.5);
            double sp = Math.sin(pitch * 0.5);
            double cr = Math.cos(roll * 0.5);
            double sr = Math.sin(roll * 0.5);


            float qua_w = (float)(cr * cp * cy + sr * sp * sy);
            float qua_x = (float)(sr * cp * cy - cr * sp * sy);
            float qua_y = (float)(cr * sp * cy + sr * cp * sy);
            float qua_z = (float)(cr * cp * sy - sr * sp * cy);

            return new Quaternion(qua_x, qua_y, qua_z, qua_w);
        }


        public double[] homogeneousTransform(double[][] matrix1, double[] matrix2) {
            double[] matrixAns = new double[4];

            //Calculation Part//
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    matrixAns[i] += matrix1[i][j] * matrix2[j];
                }
            }
            // debug //
            //System.out.println(Arrays.toString(matrixAns));
            return new double[] {matrixAns[0] , matrixAns[1], matrixAns[2]};
        }

        public Quaternion rotationCalculator(double x, double y, double z, double xp, double yp,double zp, double addY)
        {
            double x_sub,y_sub,z_sub,x_deg,z_deg;
            x_sub  = xp-x; // + is right // - is left
            y_sub = Math.abs(yp-y); // always absolute

            double bftan = x_sub/y_sub;
            z_deg = Math.toDegrees(Math.atan(bftan));
            double z_deg_final = z_deg - 90;
           // System.out.println("z deg: " + z_deg);
            z_sub = zp-z; // + is down // - is up
            bftan = z_sub/y_sub;
            x_deg = -Math.toDegrees(Math.atan(bftan));
           // System.out.println("x deg: " + x_deg);
            z_deg = Math.toRadians(z_deg_final);
            x_deg = Math.toRadians(x_deg);

            double cy = Math.cos(z_deg * 0.5);
            double sy = Math.sin(z_deg * 0.5);
            double cp = Math.cos(addY * 0.5);
            double sp = Math.sin(addY * 0.5);
            double cr = Math.cos(x_deg * 0.5);
            double sr = Math.sin(x_deg * 0.5);

            double qua_w = cr * cp * cy + sr * sp * sy;
            double qua_x = sr * cp * cy - cr * sp * sy;
            double qua_y = -(cr * sp * cy + sr * cp * sy); //zyx
            double qua_z = cr * cp * sy - sr * sp * cy;
            Quaternion q = new Quaternion((float)qua_x,(float)qua_y,(float)qua_z,(float)qua_w);
            return q;
        }
    }

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


}

