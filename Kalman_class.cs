using System;
using MathNet.Numerics.LinearAlgebra.Single;

public static class Kalman_class
{
    public struct Parameters
    {
        public float dT;
        public float declination;
        public float g;
        public float accl_threshold;
        public float field;
        public float field_threshold;
        public DenseVector accl_coefs;
        public DenseVector magn_coefs;
        public DenseVector gyro_coefs;
        public float accl_noise;
        public float magn_noise;
        public float scale_noise;
        public float skew_noise;


        public Parameters(DenseVector Accl_coefs, DenseVector Magn_coefs, DenseVector Gyro_coefs)
        {
            dT = (float)1/100;
            declination = (float) 0;
            g = (float) 1;
            accl_threshold = (float) 1;
            field = (float) 1;
            field_threshold = (float) 1;
            accl_coefs = Accl_coefs;
            magn_coefs = Magn_coefs;
            gyro_coefs = Gyro_coefs;
            accl_noise = (float) Math.Pow(10, 2);
            magn_noise = (float)Math.Pow(10, 2);
            scale_noise = (float) Math.Pow(10,-15);
            skew_noise = (float) Math.Pow(10,-15);
        }
    }

    public struct Sensors
    {
        public Matrix w;
        public Matrix a;
        public Matrix m;

        public Sensors(Matrix W, Matrix A, Matrix M)
        {
            w = W;
            a = A;
            m = M;
        }
    }

    public struct State
    {
        public Matrix R;
        public Matrix Q;
        public Matrix P;
        public Matrix dw;
        public Matrix dB;
        public Matrix dG;
        public Matrix q;

        public State(float accl_noise, float magn_noise, float angle_noise, float bias_noise, float scale_noise, float skew_noise, Matrix Quat)
        {
            R = new DiagonalMatrix(3, 3, accl_noise);
            
            R.At(2, 2, magn_noise);
            
            Q = new DiagonalMatrix(15, 15, skew_noise);
            
            Q.At(0, 0, angle_noise);
            Q.At(1, 1, angle_noise);
            Q.At(2, 2, angle_noise);

            Q.At(3, 3, bias_noise);
            Q.At(4, 4, bias_noise);
            Q.At(5, 5, bias_noise);

            Q.At(6, 6, scale_noise);
            Q.At(7, 7, scale_noise);
            Q.At(8, 8, scale_noise);

            P = new DiagonalMatrix(15, 15, (float) Math.Pow(10, -8));

            P.At(0, 0, (float)Math.Pow(10, -1));
            P.At(1, 1, (float)Math.Pow(10, -1));
            P.At(2, 2, (float)Math.Pow(10, -1));

            P.At(3, 3, (float)Math.Pow(10, -3));
            P.At(4, 4, (float)Math.Pow(10, -3));
            P.At(5, 5, (float)Math.Pow(10, -3));

            dw = new DenseMatrix(3, 1, 0);

            dB = new DenseMatrix(3, 1, 0);

            dG = new DenseMatrix(6, 1, 0);

            q = Quat;          
        }
    }


    public static Tuple <Vector, Sensors, State> AHRS_LKF_EULER(Sensors Sense, State State, Parameters Param)
    {
        Vector Attitude = new DenseVector(6, 0);
        
        // get sensor data
        Matrix m = Sense.m;
        Matrix a = Sense.a;
        Matrix w = Sense.w;    
    
        // Correct magntometers using callibration coefficients
        Matrix B = new DenseMatrix(3,3);
        B.At(0, 0, Param.magn_coefs.At(0));
        B.At(0, 1, Param.magn_coefs.At(3));
        B.At(0, 2, Param.magn_coefs.At(4));
        B.At(1, 0, Param.magn_coefs.At(5));
        B.At(1, 1, Param.magn_coefs.At(1));
        B.At(1, 2, Param.magn_coefs.At(6));
        B.At(2, 0, Param.magn_coefs.At(7));
        B.At(2, 1, Param.magn_coefs.At(8));
        B.At(2, 2, Param.magn_coefs.At(2));
        Matrix B0 = new DenseMatrix(3, 1);
        B0.At(0, 0, Param.magn_coefs.At(9));
        B0.At(1, 0, Param.magn_coefs.At(10));
        B0.At(2, 0, Param.magn_coefs.At(11));        

        m = Matrix_Transpose(Matrix_Mult(Matrix_Minus(new DiagonalMatrix(3,3,1),B),Matrix_Minus(Matrix_Transpose(m),B0)));

        // Correct accelerometers using callibration coefficients
        B.At(0, 0, Param.accl_coefs.At(0));
        B.At(0, 1, Param.accl_coefs.At(3));
        B.At(0, 2, Param.accl_coefs.At(4));
        B.At(1, 0, Param.accl_coefs.At(5));
        B.At(1, 1, Param.accl_coefs.At(1));
        B.At(1, 2, Param.accl_coefs.At(6));
        B.At(2, 0, Param.accl_coefs.At(7));
        B.At(2, 1, Param.accl_coefs.At(8));
        B.At(2, 2, Param.accl_coefs.At(2));

        B0.At(0, 0, Param.accl_coefs.At(9));
        B0.At(1, 0, Param.accl_coefs.At(10));
        B0.At(2, 0, Param.accl_coefs.At(11));

        a = Matrix_Transpose(Matrix_Mult(Matrix_Minus(new DiagonalMatrix(3, 3, 1), B), Matrix_Minus(Matrix_Transpose(a), B0)));

        // Correct gyroscopes using callibration coefficients
        B.At(0, 0, Param.gyro_coefs.At(0));
        B.At(0, 1, Param.gyro_coefs.At(3));
        B.At(0, 2, Param.gyro_coefs.At(4));
        B.At(1, 0, Param.gyro_coefs.At(5));
        B.At(1, 1, Param.gyro_coefs.At(1));
        B.At(1, 2, Param.gyro_coefs.At(6));
        B.At(2, 0, Param.gyro_coefs.At(7));
        B.At(2, 1, Param.gyro_coefs.At(8));
        B.At(2, 2, Param.gyro_coefs.At(2));

        B0.At(0, 0, Param.gyro_coefs.At(9));
        B0.At(1, 0, Param.gyro_coefs.At(10));
        B0.At(2, 0, Param.gyro_coefs.At(11));

        w = Matrix_Transpose(Matrix_Mult(Matrix_Minus(new DiagonalMatrix(3, 3, 1), B), Matrix_Minus(Matrix_Transpose(w), B0)));

        // Get State
        Matrix q = State.q;
        Matrix dB = State.dB;
        Matrix dG = State.dG;
        Matrix dw = State.dw;
        Matrix P = State.P;

        Matrix Wb = Matrix_Transpose(w);
        Matrix Ab = Matrix_Transpose(a);
        Matrix Mb = Matrix_Transpose(m);

        float dT = Param.dT;
        
        //Correct Gyroscopes for estimate biases and scale factor
        B.At(0, 0, dB.At(0, 0));
        B.At(0, 1, dG.At(0, 0));
        B.At(0, 2, dG.At(1, 0));
        B.At(1, 0, dG.At(2, 0));
        B.At(1, 1, dB.At(1, 0));
        B.At(1, 2, dG.At(3, 0));
        B.At(2, 0, dG.At(4, 0));
        B.At(2, 1, dG.At(5, 0));
        B.At(2, 2, dB.At(2, 0));

        Matrix Omegab_ib;
        Omegab_ib = Matrix_Minus(Matrix_Mult(Matrix_Minus(new DiagonalMatrix(3, 3, 1), B), Wb),dw);
        
        //Quternion calculation
        q = mrotate(q, Omegab_ib, dT);

        //DCM calculation
        Matrix Cbn = quat_to_DCM(q);
        //Gyro Angles
        Matrix angles = dcm2angle(Cbn);
        float Psi = (float) angles.At(0, 0);
        float Theta = (float) angles.At(0, 1);
        float Gamma = (float) angles.At(0, 2);

        //Acceleration Angles
        float ThetaAcc = (float)Math.Atan2(Ab.At(0, 0), Math.Sqrt(Ab.At(1, 0) * Ab.At(1, 0) + Ab.At(2, 0) * Ab.At(2, 0)));
        float GammaAcc = (float)-Math.Atan2(Ab.At(1, 0), Math.Sqrt(Ab.At(0, 0) * Ab.At(0, 0) + Ab.At(2, 0) * Ab.At(2, 0)));
        
        //Horizontal projection of magnetic field  
        angles.At(0, 0, 0);
        angles.At(0, 1, ThetaAcc);
        angles.At(0, 2, GammaAcc);
        Matrix Cbh = angle2dcm(angles);
        Matrix Mh = Matrix_Mult(Matrix_Transpose(Cbh), Mb);

        //Magnetic Heading
        float PsiMgn = (float)(-Math.Atan2(Mh.At(1,0),Mh.At(0,0)) + Param.declination);

        //System matrix
        Matrix A = new DenseMatrix (15, 15, 0);
        Matrix I = new DiagonalMatrix (15, 15, 1);

        A.At(0, 3, 1);
        A.At(1, 4, 1);
        A.At(2, 5, 1);

        A.At(0, 6, 1);
        A.At(1, 7, 1);
        A.At(2, 8, 1);

        A.At(0, 9, Omegab_ib.At(1, 0));
        A.At(0, 10, Omegab_ib.At(2, 0));
        A.At(1, 11, Omegab_ib.At(0, 0));                
        A.At(1, 12, Omegab_ib.At(2, 0));
        A.At(2, 13, Omegab_ib.At(0, 0));
        A.At(2, 14, Omegab_ib.At(1, 0));

        Matrix F = Matrix_Plus(I, Matrix_Const_Mult(A,dT));

        //Measurment Matrix
        float dPsi = Psi - PsiMgn;
        if (dPsi > Math.PI) dPsi = (float)(dPsi - 2 * Math.PI);
        if (dPsi < -Math.PI) dPsi = (float)(dPsi + 2 * Math.PI);

        float dTheta = Theta - ThetaAcc;
        if (dTheta > Math.PI) dTheta = (float)(dTheta - 2 * Math.PI);
        if (dTheta < -Math.PI) dTheta = (float)(dTheta + 2 * Math.PI);

        float dGamma = Gamma - GammaAcc;
        if (dGamma > Math.PI) dGamma = (float)(dGamma - 2 * Math.PI);
        if (dGamma < -Math.PI) dGamma = (float)(dGamma + 2 * Math.PI);        

        Matrix z = new DenseMatrix(3, 1, 1);

        z.At(0, 0, dGamma);
        z.At(1, 0, dTheta);
        z.At(2, 0, dPsi);

        Matrix H = new DenseMatrix(3, 15, 0);
        H.At(0, 0, 1);
        H.At(1, 1, 1);
        H.At(2, 2, 1);

        //Kalman Filter
        Matrix Q = State.Q;
        Matrix R = State.R;

        P = Matrix_Plus(Matrix_Mult(Matrix_Mult(F, P), Matrix_Transpose(F)),Q);
        Tuple<Matrix, Matrix> KF_result;
        KF_result = KF_Cholesky_update(P,z,R,H);
        Matrix xf = KF_result.Item1;
        P = KF_result.Item2;        

        Matrix df_hat = new DenseMatrix(3, 1, 0);
        df_hat.At(0, 0, xf.At(0, 0));
        df_hat.At(1, 0, xf.At(1, 0));
        df_hat.At(2, 0, xf.At(2, 0));

        Matrix dw_hat = new DenseMatrix(3, 1, 0);
        dw_hat.At(0, 0, xf.At(3, 0));
        dw_hat.At(1, 0, xf.At(4, 0));
        dw_hat.At(2, 0, xf.At(5, 0));
        Matrix dB_hat = new DenseMatrix(3, 1, 0);
        dB_hat.At(0, 0, xf.At(6, 0));
        dB_hat.At(1, 0, xf.At(7, 0));
        dB_hat.At(2, 0, xf.At(8, 0));
        Matrix dG_hat = new DenseMatrix(6, 1, 0);
        dG_hat.At(0, 0, xf.At(9, 0));
        dG_hat.At(1, 0, xf.At(10, 0));
        dG_hat.At(2, 0, xf.At(11, 0));
        dG_hat.At(3, 0, xf.At(12, 0));
        dG_hat.At(4, 0, xf.At(13, 0));
        dG_hat.At(5, 0, xf.At(14, 0));

        dw = Matrix_Plus(dw, dw_hat);
        dB = Matrix_Plus(dB, dB_hat);
        dG = Matrix_Plus(dG, dG_hat);

        Matrix dCbn = new DenseMatrix(3, 3, 0);

        dCbn.At(0, 1, -df_hat.At(2, 0));
        dCbn.At(0, 2, df_hat.At(1, 0));
        dCbn.At(1, 0, df_hat.At(2, 0));
        dCbn.At(1, 2, -df_hat.At(0, 0));
        dCbn.At(2, 0, -df_hat.At(1, 0));
        dCbn.At(2, 1, df_hat.At(0, 0));

        Cbn = Matrix_Mult(Matrix_Plus(new DiagonalMatrix(3, 3, 1), dCbn), Cbn);        

        q = dcm2quat(Cbn);
        q = quat_norm(q);
        
        Attitude.At(0, Psi);
        Attitude.At(1, Theta);
        Attitude.At(2, Gamma);
        Attitude.At(3, PsiMgn);
        Attitude.At(4, ThetaAcc);
        Attitude.At(5, GammaAcc);

        State.q = q;
        State.dG = dG;
        State.dB = dB;
        State.dw = dw;
        State.P = P;

        Sense.w = Matrix_Transpose(Omegab_ib);
        Sense.a = a;
        Sense.m = m;

        return new Tuple <Vector, Sensors, State>(Attitude, Sense, State); 
    }

    public static Matrix Matrix_Minus(Matrix A, Matrix B)
    {
        Matrix C = new DenseMatrix(A.RowCount,A.ColumnCount,0);
        int i, j;
        if (A.RowCount != B.RowCount | A.ColumnCount != B.ColumnCount) return C;
        for (i = 0; i < A.RowCount; i++)
        {
            for (j = 0; j < A.ColumnCount; j++)
            {
                C.At(i,j,A.At(i,j)-B.At(i,j));
            }
        }
            return C;
    }

    public static Matrix Matrix_Plus(Matrix A, Matrix B)
    {
        Matrix C = new DenseMatrix(A.RowCount, A.ColumnCount, 0);
        int i, j;
        if (A.RowCount != B.RowCount | A.ColumnCount != B.ColumnCount) return C;
        for (i = 0; i < A.RowCount; i++)
        {
            for (j = 0; j < A.ColumnCount; j++)
            {
                C.At(i, j, A.At(i, j) + B.At(i, j));
            }
        }
        return C;
    }

    public static Matrix Matrix_Const_Mult(Matrix A, float B)
    {
        Matrix C = new DenseMatrix(A.RowCount, A.ColumnCount, 0);
        int i, j;
        for (i = 0; i < A.RowCount; i++)
        {
            for (j = 0; j < A.ColumnCount; j++)
            {
                C.At(i, j, A.At(i, j)*B);
            }
        }
        return C;
    }

    public static Matrix Matrix_Mult(Matrix A, Matrix B)
    {
        Matrix C = new DenseMatrix(A.RowCount, B.ColumnCount, 0);
        int i, j, k;
        float el;
        if (A.ColumnCount != B.RowCount) return C;
        for (i = 0; i < A.RowCount; i++)
        {
            for (j = 0; j < B.ColumnCount; j++)
            {
                el = 0;
                for (k = 0; k < B.RowCount; k++)
                {
                    el = el + A.At(i, k) * B.At(k, j);
                }
                C.At(i, j, el);
            }
        }
        return C;
    }

    public static Matrix Matrix_Transpose(Matrix A)
    {
        Matrix C = new DenseMatrix(A.ColumnCount, A.RowCount);
        int i, j;
        for (i = 0; i < A.RowCount; i++)
        {
            for (j = 0; j < A.ColumnCount; j++)
            {
                C.At(j, i, A.At(i, j));
            }
        }
        return C;
    }

    public static Matrix mrotate(Matrix q, Matrix w, float dT)
    {
        float Fx = w.At(0, 0) * dT;
        float Fy = w.At(1, 0) * dT;
        float Fz = w.At(2, 0) * dT;
        float Fm = (float)Math.Sqrt(Fx * Fx + Fy * Fy + Fz * Fz);
        float sinFm2 = (float)Math.Sin(Fm / 2);
        float cosFm2 = (float)Math.Cos(Fm / 2);
        Matrix Nd = new DenseMatrix(1, 4, 0);
        if (Fm != (float)0)
        {
            Nd.At(0, 0, cosFm2);
            Nd.At(0, 1, (Fx / Fm) * sinFm2);
            Nd.At(0, 2, (Fy / Fm) * sinFm2);
            Nd.At(0, 3, (Fz / Fm) * sinFm2);
        }
        else Nd.At(0, 0, 1);
        q = quat_multi(q, Nd);
        return q;
    }

    public static Matrix quat_multi(Matrix A, Matrix B)
    {
        Matrix C = new DenseMatrix(A.RowCount, A.ColumnCount, 0);
        if (A.RowCount != 1 | A.ColumnCount != 4) return C;
        if (B.RowCount != 1 | B.ColumnCount != 4) return C;
        C.At(0, 0, A.At(0, 0) * B.At(0, 0) - B.At(0, 1) * A.At(0, 1) - B.At(0, 2) * A.At(0, 2) - B.At(0, 3) * A.At(0, 3));
        C.At(0, 1, A.At(0, 0) * B.At(0, 1) + B.At(0, 0) * A.At(0, 1) + B.At(0, 3) * A.At(0, 2) - B.At(0, 2) * A.At(0, 3));
        C.At(0, 2, A.At(0, 0) * B.At(0, 2) + B.At(0, 0) * A.At(0, 2) - B.At(0, 3) * A.At(0, 1) + B.At(0, 1) * A.At(0, 3));
        C.At(0, 3, A.At(0, 0) * B.At(0, 3) + B.At(0, 2) * A.At(0, 1) - B.At(0, 1) * A.At(0, 2) + B.At(0, 0) * A.At(0, 3));
        return C;
    }

    public static Matrix quat_to_DCM(Matrix q)
    {
        Matrix DCM = new DenseMatrix(3, 3, 0);
        q = quat_norm(q);
        DCM.At(0, 0, (q.At(0, 0) * q.At(0, 0) + q.At(0, 1) * q.At(0, 1) - q.At(0, 2) * q.At(0, 2) - q.At(0, 3) * q.At(0, 3)));
        DCM.At(0, 1, 2 * (q.At(0, 1) * q.At(0, 2) + q.At(0, 0) * q.At(0, 3)));
        DCM.At(0, 2, 2 * (q.At(0, 1) * q.At(0, 3) - q.At(0, 0) * q.At(0, 2)));
        DCM.At(1, 0, 2 * (q.At(0, 1) * q.At(0, 2) - q.At(0, 0) * q.At(0, 3)));
        DCM.At(1, 1, (q.At(0, 0) * q.At(0, 0) - q.At(0, 1) * q.At(0, 1) + q.At(0, 2) * q.At(0, 2) - q.At(0, 3) * q.At(0, 3)));
        DCM.At(1, 2, 2 * (q.At(0, 2) * q.At(0, 3) + q.At(0, 0) * q.At(0, 1)));
        DCM.At(2, 0, 2 * (q.At(0, 1) * q.At(0, 3) + q.At(0, 0) * q.At(0, 2)));
        DCM.At(2, 1, 2 * (q.At(0, 2) * q.At(0, 3) - q.At(0, 0) * q.At(0, 1)));
        DCM.At(2, 2, (q.At(0, 0) * q.At(0, 0) - q.At(0, 1) * q.At(0, 1) - q.At(0, 2) * q.At(0, 2) + q.At(0, 3) * q.At(0, 3)));
        return DCM;
    }

    public static Matrix quat_norm(Matrix q)
    {
        float Mod = (float) Math.Sqrt(q.At(0, 0) * q.At(0, 0) + q.At(0, 1) * q.At(0, 1) + q.At(0, 2) * q.At(0, 2) + q.At(0, 3) * q.At(0, 3));        
        q.At(0, 0, q.At(0, 0) / Mod);
        q.At(0, 1, q.At(0, 1) / Mod);
        q.At(0, 2, q.At(0, 2) / Mod);
        q.At(0, 3, q.At(0, 3) / Mod);
        return q;
    }

    public static Matrix dcm2angle(Matrix DCM)
    {
        Matrix angles = new DenseMatrix(1,3,0);
        if (DCM.RowCount != 3 | DCM.ColumnCount != 3) return angles;
        angles.At(0, 0, (float)(Math.Atan2(DCM.At(0, 1), DCM.At(0, 0))));
        angles.At(0, 1, (float)-(Math.Atan2(DCM.At(0, 2), Math.Sqrt(1 - DCM.At(0, 2) * DCM.At(0, 2)))));
        angles.At(0, 2, (float)(Math.Atan2(DCM.At(1, 2), DCM.At(2, 2))));
        return angles;
    }

    public static Matrix angle2dcm(Matrix angles)
    {
        Matrix DCM = new DenseMatrix(3, 3, 0);
        if (angles.RowCount != 1 | angles.ColumnCount != 3) return DCM;
        float Cos1 = (float)Math.Cos(angles.At(0, 0));
        float Cos2 = (float)Math.Cos(angles.At(0, 1));
        float Cos3 = (float)Math.Cos(angles.At(0, 2));
        float Sin1 = (float)Math.Sin(angles.At(0, 0));
        float Sin2 = (float)Math.Sin(angles.At(0, 1));
        float Sin3 = (float)Math.Sin(angles.At(0, 2));
        DCM.At(0, 0, Cos2 * Cos1);
        DCM.At(0, 1, Cos2 * Sin1);
        DCM.At(0, 2, -Sin2);
        DCM.At(1, 0, Sin3 * Sin2 * Cos1 - Cos3 * Sin1);
        DCM.At(1, 1, Sin3 * Sin2 * Sin1 + Cos3 * Cos1);
        DCM.At(1, 2, Sin3 * Cos2);
        DCM.At(2, 0, Cos3 * Sin2 * Cos1 + Sin3 * Sin1);
        DCM.At(2, 1, Cos3 * Sin2 * Sin1 - Sin3 * Cos1);
        DCM.At(2, 2, Cos3 * Cos2);
        return DCM;
    }

    public static Matrix dcm2quat(Matrix DCM)
    {
        Matrix q = new DenseMatrix(1, 4, 0);
        float tr = trace (DCM);
        float sqtrp1 = (float)Math.Sqrt(tr + 1);
        q.At(0, 0, (float)0.5*sqtrp1);
        q.At(0, 1, (DCM.At(1, 2) - DCM.At(2, 1)) / (2 * sqtrp1));
        q.At(0, 2, (DCM.At(2, 0) - DCM.At(0, 2)) / (2 * sqtrp1));
        q.At(0, 3, (DCM.At(0, 1) - DCM.At(1, 0)) / (2 * sqtrp1));
        return q;
    }

    public static float trace (Matrix A)
    {
        float tr = (float)A.At(0, 0) + A.At(1, 1) + A.At(2, 2);
        return tr;
    }

    public static Tuple<Matrix, Matrix> KF_Cholesky_update(Matrix P, Matrix v, Matrix R, Matrix H)
    {        
        Matrix PHt = Matrix_Mult(P, Matrix_Transpose(H));
        Matrix S = Matrix_Plus(Matrix_Mult(H, PHt), R);
        S = Matrix_Const_Mult(Matrix_Plus(S, Matrix_Transpose(S)), (float)0.5);
        var SChol = S.Cholesky();
        var SCholInv = SChol.Factor.Inverse();
        Matrix W1 = Matrix_Mult(PHt, (Matrix) SCholInv);
        Matrix W = Matrix_Mult(W1,Matrix_Transpose((Matrix) SCholInv));
        Matrix x = Matrix_Mult(W,v);
        P = Matrix_Minus(P, Matrix_Mult(W1, Matrix_Transpose(W1)));
        return new Tuple<Matrix, Matrix>(x, P);        
    }

}
