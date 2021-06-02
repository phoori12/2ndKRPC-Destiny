package jp.jaxa.iss.kibo.rpc.defaultapk;

import java.util.ArrayList;
import java.util.List;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.calib3d.*;

public class DestinyCore {

        public double[] DetectAR(Mat img, Mat cameraMatrix, Mat distCoeffs) {
            Mat ids = new Mat();
            Mat tvecs = new Mat();
            Mat rvecs = new Mat();
            List<Mat> corners = new ArrayList<>();
            List<Mat> rej = new ArrayList<>();

            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            DetectorParameters param = DetectorParameters.create();
            Aruco.detectMarkers(img, dictionary, corners, ids, param, rej, cameraMatrix, distCoeffs);
//		Got corners and ids
            System.out.println("ids: " + "\n" + ids.dump());
            Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);
            System.out.println("rvecs" + rvecs.dump());
            System.out.println("tvecs" + tvecs.dump());
            System.out.println("");
//		Got tvecs and rvecs

            Mat _tvecs = new Mat(1, 3, CvType.CV_64FC1);
            Mat _rvecs = new Mat(1, 3, CvType.CV_64FC1);
            Mat rot = new Mat();
            double[] result = new double[4];
            double[][] ID = new double[4][1];
            double[][] matrixAns = new double[4][1];
            ArrayList<double[][]> tuples = new ArrayList<double[][]>();

            for (int i = 0; i < ids.rows(); i++) {
                System.out.println("Ids " + ids.get(i, 0)[0] + ": ");
                ID[i][0] = ids.get(i, 0)[0];
                _rvecs.put(0, 0, rvecs.get(i, 0)); //i  0
                _tvecs.put(0, 0, tvecs.get(i, 0)); //i  0
                System.out.println("_rvecs: " + _rvecs.dump());
                System.out.println("_tvecs: " + _tvecs.dump());
                Calib3d.Rodrigues(_rvecs, rot);
                System.out.println("rot: " + rot.dump());
                matrixAns = homoCal((int) ID[i][0], rot, _tvecs);
                tuples.add(matrixAns);
                System.out.println(tuples.size() + "in list");
            }
            //Debug//
            for (int a = 0; a < 4; a++) {
                for (int b = 0; b < 4; b++) {
                    System.out.println("Tuples: " + tuples.get(a)[b][0]);
                }
            }
            System.out.println("TupleSize: " + tuples.size());
            double[][] result_avg = new double[4][1];
            result_avg = average(tuples);
//		System.out.println("Result_avg: " + result_avg);
            return result;
        }

    public double[][] homoCal(int ids,Mat rot, Mat tvecs)
    {
        int row, col, a;
        double[][] matrix1 = new double[4][4];
        double[][] matrix2 = new double[4][1];
        double[][] matrixAns = new double[4][1];
        double xbar = 0.1125;
        double ybar = 0;
        double zbar = 0.0415;
        System.out.println("ID: " + ids);
//      matrix1[0][0] = rot.get(0, 0)[0]; matrix1[0][1] = rot.get(0, 0)[0]; matrix1[0][2] = rot.get(0, 0)[0]; matrix1[0][3] = dx;
//    	matrix1[1][0] = rot.get(0, 0)[0]; matrix1[1][1] = rot.get(0, 0)[0]; matrix1[1][2] = rot.get(0, 0)[0]; matrix1[1][3] = dy;
//    	matrix1[2][0] = rot.get(0, 0)[0]; matrix1[2][1] = rot.get(0, 0)[0]; matrix1[2][2] = rot.get(0, 0)[0]; matrix1[2][3] = dz;
        matrix1[0][3] = 0; matrix1[1][3] = 0; matrix1[2][3] = 0; matrix1[3][3] = 1;
        matrix1[0][3] = tvecs.get(0, 0)[0];
        matrix1[1][3] = tvecs.get(0, 1)[0];
        matrix1[2][3] = tvecs.get(0, 2)[0];

        if(ids == 3)
        {
            matrix2[0][0] = xbar;
            matrix2[1][0] = ybar;
            matrix2[2][0] = -zbar;
            matrix2[3][0] = 1;
            System.out.println("Matrix2_ID" + ids + ": " + matrix2[2][0]);
            for(row = 0; row < 3; row++)
            {
                for(col = 0; col < 3; col++)
                {
                    matrix1[row][col] = rot.get(row, col)[0];
//	    			System.out.println(matrix1[row][col]);
                }
            }
        }
        else if(ids == 4)
        {
            matrix2[0][0] = -xbar;
            matrix2[1][0] = ybar;
            matrix2[2][0] = -zbar;
            matrix2[3][0] = 1;
            System.out.println("Matrix2_ID" + ids + ": " + matrix2[2][0]);

            for(row = 0; row < 3; row++)
            {
                for(col = 0; col < 3; col++)
                {
                    matrix1[row][col] = rot.get(row, col)[0];
//	    			System.out.println(matrix1[row][col]);
                }
            }
        }
        else if(ids == 2)
        {

            matrix2[0][0] = xbar;
            matrix2[1][0] = ybar;
            matrix2[2][0] = zbar;
            matrix2[3][0] = 1;
            System.out.println("Matrix2_ID" + ids + ": " + matrix2[2][0]);

            for(row = 0; row < 3; row++)
            {
                for(col = 0; col < 3; col++)
                {
                    matrix1[row][col] = rot.get(row, col)[0];
//	    			System.out.println(matrix1[row][col]);
                }
            }
        }
        else if(ids == 1)
        {
            matrix2[0][0] = -0.1125;
            matrix2[1][0] = 0;
            matrix2[2][0] = 0.0415;
            matrix2[3][0] = 1;
            System.out.println("Matrix2_ID" + ids + ": " + matrix2[2][0]);

            for(row = 0; row < 3; row++)
            {
                for(col = 0; col < 3; col++)
                {
                    matrix1[row][col] = rot.get(row, col)[0];
//	    			System.out.println(matrix1[row][col]);
                }
            }
        }
        else
        {
            System.out.println("Excessed");
        }

        //Calculation Part//
        for (row = 0; row < 4; row++)
        {
            for (col = 0; col < 4; col++)
            {
                matrixAns[row][0] += matrix1[row][col] * matrix2[col][0];
            }
        }

        ////////DEBUG////////
        //Matrix1//
        System.out.println("");
        System.out.println("Matrix1: ");
        for(row = 0; row < 4; row++)
        {
            for(col = 0; col < 4; col++) { }
            System.out.println(matrix1[row][0] + "    " + matrix1[row][1] + "    " + matrix1[row][2] + "    " + matrix1[row][3]);
        }

        //Matrix2//
        System.out.println("");
        System.out.println("Matrix2: ");
        for(row = 0; row < 4; row++)
        {
            for(col = 0; col < 1; col++) { }
            System.out.println(matrix2[row][0]);
        }

        //MatrixAns//
        System.out.println("");
        System.out.println("MatrixAns: ");
        for (row = 0; row < 4; row++)
        {
            for (col = 0; col < 1; col++)
            {
                System.out.print(matrixAns[row][col]);
            }
            System.out.println("");
        }

        return matrixAns;
    }

    public double[][] average(ArrayList<double[][]> tuples)
    {
        double[][] result_avg = new double[4][1];
//		result_avg[0][0] = tuples.get(0)[0][0] + tuples.get(1)[0][0] + tuples.get(2)[0][0] + tuples.get(3)[0][0];
//		result_avg[1][0] = tuples.get(0)[1][0] + tuples.get(1)[1][0] + tuples.get(2)[1][0] + tuples.get(3)[1][0];
//		result_avg[2][0] = tuples.get(0)[2][0] + tuples.get(1)[2][0] + tuples.get(2)[2][0] + tuples.get(3)[2][0];
//		result_avg[3][0] = tuples.get(0)[3][0] + tuples.get(1)[3][0] + tuples.get(2)[3][0] + tuples.get(3)[3][0];

        for(int i = 0; i < tuples.size(); i++)
        {
            result_avg[i][0] = (tuples.get(0)[i][0] + tuples.get(1)[i][0] + tuples.get(2)[i][0] + tuples.get(3)[i][0]) / 4;
            System.out.println("result_avg: " + result_avg[i][0]);
        }
        return result_avg;
    }


}
