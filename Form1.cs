using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using MathNet.Numerics.LinearAlgebra.Single;


namespace logReader
{
    public partial class Form1 : Form
    {
        string file2read;
        string file2save = "";
        


        public Form1()
        {
            InitializeComponent();
        }

        private void fileButton_Click(object sender, EventArgs e)
        {
            openFileDialog.InitialDirectory = "";
            openFileDialog.FileName = "";
            openFileDialog.Filter = "Доступные форматы (*.log; *.txt)|*.log; *.txt|Все файлы (*.*)|*.*";
            openFileDialog.Title = "Выберите файл для чтения";
            DialogResult result = openFileDialog.ShowDialog();
            if (result == DialogResult.OK)
            {
                string[] file_name_parts = openFileDialog.FileName.Split('\\');
                fileBox.Text = file_name_parts[file_name_parts.Length - 1];
                fileBox.Update();
                file2read = openFileDialog.FileName;
                readButton.Enabled = true;
            }
            else
            {
                fileBox.Text = "Неверно задан файл";
                readButton.Enabled = false;
            }
        }

        private void readButton_Click(object sender, EventArgs e)
        {
            saveFileDialog.InitialDirectory = "";
            saveFileDialog.FileName = "";
            saveFileDialog.Filter = "Все файлы (*.*)|*.*";
            saveFileDialog.Title = "Выберите файл для сохранения данных";
            DialogResult result = saveFileDialog.ShowDialog();
            if (result == DialogResult.OK)
            {
                if (saveFileDialog.FileName.Contains('.'))
                {
                    string[] file_name_parts = saveFileDialog.FileName.Split('.');
                    file2save = "";

                    for (int i = 0; i < file_name_parts.Length - 1; i++)
                        file2save += file_name_parts[i] + ".";
                }
                else
                    file2save = saveFileDialog.FileName + ".";
                read_file();
            }
            else
            {
                MessageBox.Show("Неверно задан файл для сохранения");
            }
        }

        private void read_file()
        {
            double[,] q;
            double[,] a;
            double[,] w;
            double[,] m;
            int k = 0, k2 = 0;
            uint[] ticks;
            uint[] ticks2;
            byte[] type;
            double[] lat;
            double[] lon;
            double[] speed;
            double[] course;
            double[] time;
            double[] stat;
            double[] date;

            byte[] full_file = File.ReadAllBytes(file2read);
            ticks = new uint[full_file.Length / 35 + 1];
            ticks2 = new uint[ticks.Length];
            type = new byte[ticks.Length];
            
            byte[] pack = new byte[32];
            byte[] pack2 = new byte[26];
            int crc;
            int tt = 0;
            a = new double[ticks.Length, 3];
            w = new double[ticks.Length, 3];
            m = new double[ticks.Length, 3];
            q = new double[ticks.Length, 4];
            int[] counter = new int[ticks.Length];
            lat = new double[ticks2.Length];
            lon = new double[ticks2.Length];
            speed = new double[ticks2.Length];
            course = new double[ticks2.Length];
            time = new double[ticks2.Length];
            stat = new double[ticks2.Length];
            date = new double[ticks2.Length];
            byte[] buffer = new byte[2];
            byte[] buffer2 = new byte[4];
            this.Update();
            for (int i = 0; i < full_file.Length - 30; i++)
            {
                if ((i < full_file.Length - 35) && (full_file[i + 34] == 3) && (full_file[i + 33] == 16) &&
                    (full_file[i] == 16) && (full_file[i + 1] == 49))
                {
                    crc = 0;
                    for (int j = 0; j < 32; j++)
                    {
                        pack[j] = full_file[i + j + 1];
                        if (j < 31)
                            crc = crc ^ pack[j];
                    }
                    if (crc == pack[pack.Length - 1])
                    {
                        //ticks[k] = pack[4] + pack[3] * (uint)Math.Pow(2, 8) + pack[2] * (uint)Math.Pow(2, 16) + pack[1] * (uint)Math.Pow(2, 24);
                        ticks[k] = BitConverter.ToUInt32(pack, 1);
                        type[k] = pack[0];
                        if (type[k] == 49)
                        {
                            buffer[0] = pack[7]; buffer[1] = pack[8];
                            a[k, 0] = ((double)BitConverter.ToInt16(buffer, 0)) * 0.0018;
                            buffer[0] = pack[5]; buffer[1] = pack[6];
                            a[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.0018;
                            buffer[0] = pack[9]; buffer[1] = pack[10];
                            a[k, 2] = -(double)BitConverter.ToInt16(buffer, 0) * 0.0018;

                            buffer[0] = pack[13]; buffer[1] = pack[14];
                            w[k, 0] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;
                            buffer[0] = pack[11]; buffer[1] = pack[12];
                            w[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;
                            buffer[0] = pack[15]; buffer[1] = pack[16];
                            w[k, 2] = (double)BitConverter.ToInt16(buffer, 0) * 0.00053375;

                            buffer[0] = pack[17]; buffer[1] = pack[18];
                            m[k, 0] = -(double)BitConverter.ToInt16(buffer, 0) * 0.00030518;
                            buffer[0] = pack[19]; buffer[1] = pack[20];
                            m[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00030518;
                            buffer[0] = pack[21]; buffer[1] = pack[22];
                            m[k, 2] = -(double)BitConverter.ToInt16(buffer, 0) * 0.00030518;

                            buffer[0] = pack[23]; buffer[1] = pack[24];
                            q[k, 0] = (double)BitConverter.ToInt16(buffer, 0);
                            buffer[0] = pack[25]; buffer[1] = pack[26];
                            q[k, 1] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                            buffer[0] = pack[27]; buffer[1] = pack[28];
                            q[k, 2] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                            buffer[0] = pack[29]; buffer[1] = pack[30];
                            q[k, 3] = (double)BitConverter.ToInt16(buffer, 0) * 0.00003125;
                        }
                        counter[k] = k2;
                        k++;
                        
                    }
                    else
                        tt++;
                }
                if ((full_file[i + 29] == 3) && (full_file[i + 28] == 16) && (full_file[i] == 16) &&
                    (full_file[i + 1] == 50))
                {
                    crc = 50;
                    for (int j = 0; j < 26; j++)
                    {
                        pack2[j] = full_file[i + j + 2];
                        if (j < 25)
                            crc = crc ^ pack2[j];
                    }
                    if (crc == pack2[pack2.Length - 1])
                    {
                        //ticks2[k2] = pack2[3] + pack2[2] * (uint)Math.Pow(2, 8) +
                        //    pack2[1] * (uint)Math.Pow(2, 16) + pack2[0] * (uint)Math.Pow(2, 24);
                        ticks2[k2] = BitConverter.ToUInt32(pack2, 0);
                        buffer2[0] = pack2[4]; buffer2[1] = pack2[5]; buffer2[2] = pack2[6]; buffer2[3] = pack2[7];
                        lat[k2] = ((double)BitConverter.ToInt32(buffer2, 0)) / 600000;
                        buffer2[0] = pack2[8]; buffer2[1] = pack2[9]; buffer2[2] = pack2[10]; buffer2[3] = pack2[11];
                        lon[k2] = ((double)BitConverter.ToInt32(buffer2, 0)) / 600000;
                        buffer[0] = pack2[12]; buffer[1] = pack2[13];
                        speed[k2] = (double)BitConverter.ToInt16(buffer, 0) / 100;
                        buffer[0] = pack2[14]; buffer[1] = pack2[15];
                        course[k2] = (double)BitConverter.ToInt16(buffer, 0) / 160;
                        buffer2[0] = pack2[16]; buffer2[1] = pack2[17]; buffer2[2] = pack2[18]; buffer2[3] = pack2[19];
                        time[k2] = ((double)BitConverter.ToInt32(buffer2, 0)) / 10;
                        stat[k2] = pack2[20];
                        buffer2[0] = pack2[21]; buffer2[1] = pack2[22]; buffer2[2] = pack2[23]; buffer2[3] = pack2[24];
                        date[k2] = ((double)BitConverter.ToInt32(buffer2, 0));
                        k2++;

                    }
                }
            }

            readButton.Enabled = false;
            fileButton.Enabled = false;
            textBox1.Enabled = false;
            readLabel.Text = "Идет чтение:";
            readLabel.Update();
            int block_index = Convert.ToInt32(textBox1.Text.ToString());
            DenseVector Magn_coefs = new DenseVector(get_magn_coefs(block_index));
            DenseVector Accl_coefs = new DenseVector(get_accl_coefs(block_index));
            DenseVector Gyro_coefs = new DenseVector(12);
            Kalman_class.Parameters Parameters = new Kalman_class.Parameters(Accl_coefs, Magn_coefs, Gyro_coefs);
            Kalman_class.Sensors Sensors = new Kalman_class.Sensors(new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0), new DenseMatrix(1, 3, 0));
            Matrix Initia_quat = new DenseMatrix(1, 4, 0);
            Initia_quat.At(0, 0, 1);
            Kalman_class.State State = new Kalman_class.State((float)Math.Pow(10, 2), (float)Math.Pow(10, 2), (float)Math.Pow(10, -3),
                (float)Math.Pow(10, -6), (float)Math.Pow(10, -15), (float)Math.Pow(10, -15), Initia_quat);
            float[] angles = new float[3];
            float[] mw, ma, mm;
            ma = new float[3];
            mw = new float[3];
            mm = new float[3];
            Tuple<Vector, Kalman_class.Sensors, Kalman_class.State> AHRS_result;

            FileStream fs_imu = File.Create(file2save + "imu", 2048, FileOptions.None);
            BinaryWriter str_imu = new BinaryWriter(fs_imu);
            FileStream fs_gps = File.Create(file2save + "gps", 2048, FileOptions.None);
            BinaryWriter str_gps = new BinaryWriter(fs_gps);
            Int16 buf16; Byte buf8; Int32 buf32;
            Double bufD; Single bufS; UInt32 bufU32;
            progressBar.Invoke(new Action(() => progressBar.Maximum = k + k2));
            progressBar.Invoke(new Action(() => progressBar.Value = 0));

            float[] magn_c = get_magn_coefs(block_index);
            float[] accl_c = get_accl_coefs(block_index);
            float[] gyro_c = new float[12];

            for (int i = 0; i < k; i++)
            {
                progressBar.Invoke(new Action(() => progressBar.Value++));

                Sensors.a.At(0, 0, (float)a[i, 0]);
                Sensors.a.At(0, 1, (float)a[i, 1]);
                Sensors.a.At(0, 2, (float)a[i, 2]);

                Sensors.w.At(0, 0, (float)w[i, 0]);
                Sensors.w.At(0, 1, (float)w[i, 1]);
                Sensors.w.At(0, 2, (float)w[i, 2]);

                Sensors.m.At(0, 0, (float)m[i, 0]);
                Sensors.m.At(0, 1, (float)m[i, 1]);
                Sensors.m.At(0, 2, (float)m[i, 2]);

                AHRS_result = Kalman_class.AHRS_LKF_EULER(Sensors, State, Parameters);

                State = AHRS_result.Item3;

                mm = single_correction(magn_c, (float)m[i, 0], (float)m[i, 1], (float)m[i, 2]);
                ma = single_correction(accl_c, (float)a[i, 0], (float)a[i, 1], (float)a[i, 2]);
                mw = single_correction(gyro_c, (float)w[i, 0], (float)w[i, 1], (float)w[i, 2]);

                angles[0] = (float)(AHRS_result.Item1.At(0) / 1.735);
                angles[1] = (float)(AHRS_result.Item1.At(1) / 1.735);
                angles[2] = (float)(AHRS_result.Item1.At(2) / 1.735);

                // IMU
                buf16 = (Int16)(angles[0] * 10000 * 100 / (180 / Math.PI));
                str_imu.Write(buf16);
                buf16 = (Int16)(angles[1] * 10000 * 100 / (180 / Math.PI));
                str_imu.Write(buf16);
                buf16 = (Int16)(angles[2] * 10000 * 100 / (180 / Math.PI));
                str_imu.Write(buf16);

                buf16 = (Int16)(mw[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mw[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mw[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(ma[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(ma[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(ma[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(mm[0] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mm[1] * 3000);
                str_imu.Write(buf16);
                buf16 = (Int16)(mm[2] * 3000);
                str_imu.Write(buf16);

                buf16 = (Int16)(q[i, 0]);
                str_imu.Write(buf16);

                //buf32 = (Int32)(counter[i]);
                buf32 = (Int32)(ticks[i]);
                str_imu.Write(buf32);

                buf8 = (Byte)(0);
                str_imu.Write(buf8);
            }
            for (int i = 0; i < k2; i++)
            {
                progressBar.Invoke(new Action(() => progressBar.Value++));
                // GPS
                bufD = (Double)(lat[i]) / ((180 / Math.PI) * 16.66);
                str_gps.Write(bufD);
                bufD = (Double)(lon[i]) / ((180 / Math.PI) * 16.66);
                str_gps.Write(bufD);
                bufD = (Double)(0);
                str_gps.Write(bufD);

                bufS = (Single)(time[i]);
                str_gps.Write(bufS);
                bufS = (Single)(speed[i]);
                str_gps.Write(bufS);
                bufS = (Single)(0);
                str_gps.Write(bufS);
                str_gps.Write(bufS);

                //bufU32 = (UInt32)(i);
                bufU32 = (UInt32)(ticks2[i]);
                str_gps.Write(bufU32);
                buf8 = (Byte)(0);
                str_gps.Write(buf8);
                str_gps.Write(buf8);
                str_gps.Write(buf8);
            }
            str_imu.Flush();
            str_imu.Close();
            str_gps.Flush();
            str_gps.Close();
            //MessageBox.Show("Сохранение завершено");
            readLabel.Text = "Чтение завершено";
            readButton.Enabled = true;
            fileButton.Enabled = true;
            textBox1.Enabled = true;
            
            

            
        }

        private float deg2rad(float degrees)
        {
            return (float)(Math.PI / 180) * degrees;
        }

        private void write_data(object Com)
        {

        }
        private float[] get_accl_coefs(int index)
        {
            float[] result = new float[0];
            switch (index)
            {
                case 1:
                    float[] temp1 = { -0.0081F, 0.0401F, -0.0089F, 0.0301F,
                                        -0.0111F, 0.0323F, 0.0093F, 0.0104F, 0.0085F, -0.0921F, -0.1201F, -0.1454F };
                    result = temp1;
                    break;
                case 2:
                    float[] temp2 = {0.043713810744819F,   0.032204099593533F,   0.014469608704782F,   0.067770825077312F,
                       0.124497968267554F,  -0.069373064241876F,  -0.020960499722065F,  -0.119156909250651F,
                       0.044421201741530F,  -0.179463499557114F,  -0.234503197266493F,  -0.036294503190156F };
                    result = temp2;
                    break;
                case 3:
                    float[] temp3 = {0.024167075678158F,   0.017921002182728F,   0.027208980951327F,   0.047646758555060F,
                      -0.052181643992751F,  -0.056215499304436F,  -0.011304687635132F,   0.087617101877347F,
                       0.051573793476529F,  -0.045096522590058F,  -0.034826544109968F,  -0.215775421842763F };
                    result = temp3;
                    break;
                case 4:
                    float[] temp4 = {0.114432559973540F,   0.106838802436507F,   0.051899182252855F,   0.042431602755402F,
                      -0.208355094982050F,  -0.058741377932612F,  -0.017284017902782F,   0.109414756776846F,
                       0.041872917797213F,  -0.207903904900104F,   0.069870156128220F,  -0.420879615848955F };
                    result = temp4;
                    break;
                case 5:
                    float[] temp5 = {-0.089083516880583F,  -0.212101554759966F,  -0.074106774400082F,   0.140475177135465F,
                       0.050748404792032F,   0.035466506409495F,   0.135105447115893F,   0.005040120014690F,
                       0.034685336520958F,  -0.105165286994008F,  -0.049062087663207F,   0.090272091944523F };
                    result = temp5;
                    break;
                case 6:
                    float[] temp6 = {-0.067103542084971F,   0.079590227298170F,   0.070742925683350F,  -0.041636392993342F,
                       0.220644176913664F,  -0.170330066249381F,   0.073407186641070F,  -0.024528044153757F,
                      -0.047531378473926F,   0.078357526154640F,  -0.155837767265987F,  -0.238145173236917F };
                    result = temp6;
                    break;
                case 7:
                    float[] temp7 = {0.045134621164588F,   0.068462421832649F,   0.017700087773030F,  -0.225994306022918F,
                       0.088191771774388F,  -0.179589945451456F,   0.025986312625294F,  -0.040027912330171F,
                      -0.046415300400838F,  -0.158947267757270F,  -0.067242068899347F,  -0.124325353336784F };
                    result = temp7;
                    break;
                case 8:
                    float[] temp8 = {0.104118389703866F,   0.084958239636849F,   0.032826888550085F,  -0.140404308281075F,
                      -0.017921707825125F,   0.061748807252006F,  -0.006715952930192F,   0.012639691500341F,
                       0.046309410792156F,  -0.218542898428085F,  -0.003731519086623F,  -0.319272823668150F };
                    result = temp8;
                    break;
                case 9:
                    float[] temp9 = {0.083867364580635F,   0.080541880516270F,   0.112368594320036F,   0.122244712046047F,
                      -0.047981420650493F,  -0.184024814423127F,  -0.097675623527326F,  -0.054026830167556F,
                       0.174687241854919F,  -0.252086667315104F,   0.043187133234385F,  -0.178080825685559F };
                    result = temp9;
                    break;
                case 10:
                    float[] temp10 = {-0.076212684578814F,   0.036874608262696F,  -0.006395329952186F,  -0.048533020632689F,
                       0.031596659443431F,   0.080190089955160F,   0.065893547405957F,  -0.066878035481739F,
                      -0.029911464522049F,  -0.031068180680396F,  -0.065093789262652F,  -0.172359413648919F };
                    result = temp10;
                    break;
                case 11:
                    float[] temp11 = {0.019329839277928F,   0.049200059518738F,   0.044353143292047F,   0.058542188048022F,
                        0.062731113213260F,  -0.028231153395039F,  -0.121639863668443F,  -0.066633118358829F,
                        0.133914377849888F,  -0.142745425008951F,  -0.028743819809420F,  -0.321475954225317F };
                    result = temp11;
                    break;
                case 12:
                    float[] temp12 = {-0.036279717606262F,   0.053297786065231F,   0.004975622946153F,   0.006037301787334F,
                       0.076916729330439F,  -0.039059863553166F,   0.022282449734369F,  -0.072682985769156F,
                       0.021034870244154F,  -0.031357531783622F,   0.048738371353602F,  -0.305991758949551F };
                    result = temp12;
                    break;
                case 13:
                    float[] temp13 = {0.022459687724866F,  -0.062077105256798F,   0.046795345083684F,  -0.281967510046409F,
                      -0.111189762502934F,   0.153912645890540F,   0.113497567735599F,  -0.236258801745200F,
                       0.107476182651660F,  -0.460519774903838F,   0.115898799892106F,  -0.013940296751586F };
                    result = temp13;
                    break;
                case 14:
                    float[] temp14 = {0.030554866588497F,   0.041807742536512F,   0.014637539550826F,   0.070444712524782F,
                      -0.045520819149987F,  -0.053792609796159F,   0.067583771817011F,   0.044201999575329F,
                      -0.068058903538097F,  -0.133268083186438F,  -0.157601990534356F,  -0.125072062660633F };
                    result = temp14;
                    break;
                case 15:
                    float[] temp15 = {-0.091419543228102F,   0.013318237063326F,   0.015108665155053F,  -0.045927518891181F,
                       0.138517055483565F,   0.115478666476207F,   0.028512863952903F,   0.154290791103919F,
                      -0.019524419888084F,  -0.171930546561895F,  -0.301610286853745F,   0.050182040765164F };
                    result = temp15;
                    break;
                default:
                    result = new float[12];
                    break;
            }

            return result;
        }

        private float[] get_magn_coefs(int index)
        {
            float[] result = new float[0];
            switch (index)
            {
                case 1:
                    float[] temp1 = {-1.3000F, -1.3499F, -1.2180F, -0.5191F, 1.2598F, 0.8968F,
                                         1.1259F, -0.8541F, -1.4552F, -0.1676F, 0.0201F, 0.1032F };
                    result = temp1;
                    break;
                case 2:
                    float[] temp2 = {-1.021584893111037F,  -1.156822840186938F,  -1.059527431706735F,   0.405595881908856F,
                       0.954589891425432F,  -0.069541017965736F,   0.991639731762399F,  -1.012866646546024F,
                      -0.886632983175849F,  -0.065839849453693F,   0.059178141113860F,   0.229560149816208F };
                    result = temp2;
                    break;
                case 3:
                    float[] temp3 = {0.238481805936342F,  -0.611518102386768F,  -0.521946604623734F,  -0.152499267950600F,
                       0.380041463141051F,  -0.089921812966795F,  -0.029707070918285F,   0.777735008725552F,
                      -0.263038218323014F,   1.102625052444242F,  -0.174805428028501F,   0.764500679478734F };
                    result = temp3;
                    break;
                case 4:
                    float[] temp4 = {-0.912310601920176F,  -0.035122859804258F,  -0.980824053980284F,  -0.900924288276854F,
                      -0.392141063030012F,  -0.212147078216013F,   0.370184540925128F,  -0.138611784797257F,
                       0.592181216595855F,  -0.716249303213732F,   0.743440070172089F,   0.417833919696198F };
                    result = temp4;
                    break;
                case 5:
                    float[] temp5 = {-0.810295610308645F,  -0.669847770840476F,   0.029900885597606F,  -0.314087876668212F,
                        -0.489991754352551F,  -0.196143690771996F,   0.785352181879690F,  -0.148859193029027F,
                        0.475841206204676F,  -0.674462559992835F,   0.797605908162643F,   1.039652155282594F };
                    result = temp5;
                    break;
                case 6:
                    float[] temp6 = {-0.170985488990754F,  -0.564445824510332F,  -0.156834833150726F,  -0.308524525175852F,
                       0.442976518336103F,  -0.358382840398978F,  -0.080578225278669F,   0.617372103949350F,
                      -0.541560455196209F,   0.718141250811105F,  -0.546677511918727F,   1.013917065538093F };
                    result = temp6;
                    break;
                case 7:
                    float[] temp7 = {0.240929547813333F,  -0.055331307635026F,   0.080962491207260F,   0.111673173134500F,
                      -0.437371531722260F,   0.086278316536676F,  -0.156009284276460F,  -0.450383773465459F,
                      -0.232586543173110F,   1.800622790457513F,   0.479391322929313F,  -1.645807763696289F };
                    result = temp7;
                    break;
                case 8:
                    float[] temp8 = {-0.657211729785750F,  -0.751518105983042F,  -0.502998987070861F,   0.346979570434948F,
                      -0.626698848495405F,   0.854645387115907F,  -0.722084569035857F,  -0.328055065021069F,
                      -0.229782299399703F,  -0.357543764086528F,  -0.658127285343566F,   0.346576778962207F };
                    result = temp8;
                    break;
                case 9:
                    float[] temp9 = {-0.626836215836308F,  -0.013759850146307F,  -0.583068885503070F,   0.859006753692338F,
                      -0.291104097898408F,   0.359496708414707F,  -0.377515731153946F,  -0.003959749268316F,
                      -0.269023628503547F,  -0.839470996073222F,  -0.734894289704346F,   0.173586855759935F };
                    result = temp9;
                    break;
                case 10:
                    float[] temp10 = {-0.369311287771329F,  -0.619181805694075F,   0.215860213252399F,  -0.202174116935395F,
                      -0.469592802232751F,  -0.047691482924571F,   0.275440905803689F,  -0.360029056603724F,
                       0.184994296407090F,  -0.770865355323793F,   0.374651486398312F,   1.142034879854652F };
                    result = temp10;
                    break;
                case 11:
                    float[] temp11 = {0.608714881216204F,   0.726988256983956F,   0.548780040935389F,   0.016242466383838F,
                       0.008309177330682F,  -0.358605622379460F,  -0.406898271841329F,  -0.176070107760827F,
                       0.023260260319988F,  -0.615605787372553F,                   0F,  -0.736861602878822F };
                    result = temp11;
                    break;
                case 12:
                    float[] temp12 = {0.174793461009216F,  -0.040792970147182F,   0.416590023305863F,   0.023815283418015F,
                      -0.400898824923381F,  -0.056977093394991F,  -0.137939234650323F,  -0.275073547098045F,
                      -0.226813306028076F,   1.386816689877866F,   0.503695987968843F,  -1.311523063280292F };
                    result = temp12;
                    break;
                case 13:
                    float[] temp13 = {-0.363402668053064F,  -1.094360523138415F,  -0.337177750046436F,  -0.150554901049520F,
                       0.760814281494252F,  -0.527695809751047F,  -0.366683816407952F,   0.694017973652759F,
                      -0.257690691379724F,   0.940590498718544F,  -0.603082996186667F,   0.915339225862064F };
                    result = temp13;
                    break;
                case 14:
                    float[] temp14 = {0.239432068058017F,  -0.725844731558349F,  -1.151204001242439F,   0.549384999741005F,
                       0.017479681993509F,   0.446775656927678F,  -0.105826078256205F,  -0.233477063739925F,
                      -0.014288927249738F,   1.072218089476175F,   0.355024708548398F,  -0.194902780695747F };
                    result = temp14;
                    break;
                case 15:
                    float[] temp15 = {0.106974544824006F,  -0.921067620149456F,  -0.544783980541099F,  -0.113463751891383F,
                       0.575096563967160F,  -0.215872226257520F,  -0.106232440232434F,   0.665018879611869F,
                      -0.120415653314238F,   0.975928019145445F,  -0.232693130860269F,   0.505738827768481F };
                    result = temp15;
                    break;
                default:
                    result = new float[12];
                    break;
            }
            return result;
        }


        private float[] single_correction(float[] coefs, float xdata, float ydata, float zdata)
        {
            float[] result = new float[3];
            Matrix B = new DiagonalMatrix(3, 3, 1);
            Matrix A = new DenseMatrix(3,3);
            A.At(0,0,coefs[0]);
            A.At(0,1,coefs[3]);
            A.At(0,2,coefs[4]);
            A.At(1,0,coefs[5]);
            A.At(1,1,coefs[1]);
            A.At(1,2,coefs[6]);
            A.At(2,0,coefs[7]);
            A.At(2,1,coefs[8]);
            A.At(2,2,coefs[2]);
            Matrix B1 = Kalman_class.Matrix_Minus(B, A);
            Matrix C = new DenseMatrix(3,1);
            C.At(0, 0, xdata);
            C.At(1, 0, ydata);
            C.At(2, 0, zdata);
            Matrix D = new DenseMatrix(3,1);
            D.At(0, 0, coefs[9]);
            D.At(1, 0, coefs[10]);
            D.At(2, 0, coefs[11]);
            Matrix res = new DenseMatrix(3,1);
            res = Kalman_class.Matrix_Mult(B1, Kalman_class.Matrix_Minus(C, D));
            result[0] = res.At(0, 0);
            result[1] = res.At(1, 0);
            result[2] = res.At(2, 0);
            return result;

        }
    }
}
