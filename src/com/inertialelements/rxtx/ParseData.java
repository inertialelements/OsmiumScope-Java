/*
* * Copyright (C) 2018 GT Silicon Pvt Ltd
 *
 * Licensed under the Creative Commons Attribution 4.0
 * International Public License (the "CCBY4.0 License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://creativecommons.org/licenses/by/4.0/legalcode
 *
 * Note that the CCBY4.0 license is applicable only for the modifications made
 * by GT Silicon Pvt Ltd
 *
*
* */
package com.inertialelements.rxtx;

import com.inertialelements.cube.JOGL3dCube;
import static com.inertialelements.rxtx.TwoWaySerialComm.dataLogger;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.swing.JFrame;

/**
 *
 * @author gts-pc-1
 */
public class ParseData {
    
    
    public static int counterNormal = 0;
    int[] header = new int[4];
    int pktNum;
    int previousPkt;
    int payloadLen;
    int startCode;
    int chkSum = 0;
    double timeStamp;
    float[] inertialData = new float[Constants.AXIS];
    public float ax;
    public float ay;
    public float az;
    public float gx;
    public float gy;
    public float gz;
    public float mx;
    public float my;
    public float mz;
    public static String ble_buffer = "";
    public static boolean normal_imu = false;
    public static int pkt_receive = 0;
    public static String df_input ="";
    float[] filter_data = new float[3];
    static public ConcurrentLinkedQueue<String> mQueue = new ConcurrentLinkedQueue<>();
    MadgwickAHRS madgwick = new MadgwickAHRS();
    JOGL3dCube panel = new JOGL3dCube();
    public static boolean time_period;
    float thetagz = 0.0f;
    

    public ParseData(final TwoWaySerialComm twoWaySerialComm) 
    {
        JFrame window = new JFrame("InertialElements is the family of high-performance motion sensor platforms");
        window.setContentPane(panel);
        window.pack();
        window.setLocation(50,50);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        window.setVisible(true);
        panel.requestFocusInWindow();
        window.addWindowListener( new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent we) {
                twoWaySerialComm.sendData(Constants.pro_off);
                twoWaySerialComm.sendData(Constants.sys_off);
                dataLogger.stopLogging();
                try {
                    dataLogger.join();
                } catch (InterruptedException ex) {
                    Logger.getLogger(ParseData.class.getName()).log(Level.SEVERE, null, ex);
                }
                System.exit(0);
            }
        } );
    }
    
    public void  get_plot_normal()
    {
        int lenght, newlen;
        if (normal_imu) {
            byte[] buffer = final_buffer(Constants.DATA_LEN);
            while (normal_imu) 
            {
//                System.out.println("pkt_receive : "+pkt_receive++);
//                System.out.println("pkt Buffer :-"+Utilities.byte2HexStr(buffer,buffer.length).replace(" ","").trim());
                parse_normal(buffer);
                String inHex = Utilities.byte2HexStr(buffer,buffer.length).replace(" ","").trim();
//                String patternStr = "AA.{4}1C";
                String patternStr = "AA";
                Pattern pattern = Pattern.compile(patternStr);
                Matcher matcher = pattern.matcher(inHex);
//                System.out.println("");
                if (startCode == 0xaa && (cal_chksum(buffer) == chkSum))
                {
                    if (previousPkt != pktNum) 
                    {
                        ax = inertialData[0];
                        ay = inertialData[1];
                        az = inertialData[2];
                        gx = inertialData[3];
                        gy = inertialData[4];
                        gz = inertialData[5];
                        if (Constants.AXIS == 9) {
                            mx = inertialData[6] * Constants.scale_pr_mag;
                            my = inertialData[7] * Constants.scale_pr_mag;
                            mz = inertialData[8] * Constants.scale_pr_mag;
                        }
                        if(Constants.check_plot)
                        {
                            filter_data = madgwick.MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
                            panel.setQuaternionData(filter_data);
                        }
                        timeStamp = pkt_receive /Constants.OUTRATE;
//                        System.out.println("previousPkt : "+previousPkt +"pktNum : " +pktNum);
                        counterNormal = 0;
                        pkt_receive++;
                        previousPkt = pktNum;
                        if (Constants.check_log && TwoWaySerialComm.dataLogger != null) 
                        {
                            dataLogger.addData(new AccGyroMag(pktNum, timeStamp, ax, ay, az, gx, gy, gz, mx, my, mz));
//                            String toFile = String.format("%12f \t %12f \t %12f \t %12f \n",filter_data[0],filter_data[1],filter_data[2],filter_data[3]);
//                            String toFile = String.format("%12s \t %12s \t %12s \t %12s \t %12s \t %12s \t %12s \t %12s \n ",String.valueOf(pktNum), dfTime.format(timeStamp), df.format(ax), df.format(ay), df.format(az), df.format(gx), df.format(gy), df.format(gz));
//                            Utilities.writeNonPDRData(TwoWaySerialComm.nonPDRFile,toFile);
                        }
                    }
                    buffer = final_buffer(Constants.DATA_LEN);
                }
                else if(matcher.find())
                {
                    int index = matcher.start();
                    String strrem = (String) inHex.subSequence(index, inHex.length());
                    lenght = strrem.length();
                    if (lenght < (Constants.DATA_LEN*2)) 
                    {
                        newlen = (Constants.DATA_LEN*2) - lenght;
                        buffer = final_buffer(newlen / 2);
                        String buffer_str = strrem + Utilities.byte2HexStr(buffer, buffer.length).replace(" ", "").trim();
                        try 
                        {
                            buffer = Utilities.hexStringToByteArray(buffer_str);
                        }
                        catch (StringIndexOutOfBoundsException e){
                            buffer = final_buffer(Constants.DATA_LEN);
                        }
                    }
                    else
                        buffer = final_buffer(Constants.DATA_LEN);
                }
                else
                {
                    buffer = final_buffer(Constants.DATA_LEN);
//                    System.out.println("new_packet "+MainActivity.byte2HexStr(buffer, buffer.length).replace(" ", "").trim());
                    counterNormal ++;
                    if (counterNormal > 5)
                    {
                        normal_imu = false;
//                        pkt_receive = 0;
                        break;
                    }
                }
                if (!normal_imu) 
                {
                    for (String mqueue_emty = mQueue.poll(); mqueue_emty != null; mqueue_emty = mQueue.poll()) 
                    {
                    }
                    break;
                }
            }
        }
    }
    
    public void parse_normal(byte[] buffer) {
        byte temp[] = new byte[4];
        int j = 0;
        if (buffer.length == Constants.DATA_LEN) {
//            System.out.println(Arrays.toString(buffer));
            for (int i = 0; i < 4; i++) {
                header[i] = buffer[j++] & 0xFF;             //header define
            }
            startCode = header[0];
//            pkt_num = (header[1] << 8) | header[2];
            pktNum = (header[1]) * 255 + header[2];
            payloadLen = header[3];

            if (startCode == 0xAA && payloadLen == 0x1F) {
                for (int i = 0; i < 4; i++) {
                    temp[i] = buffer[j++];             //time_stamp define
                }

            timeStamp = (ByteBuffer.wrap(temp).getFloat())/64e6d;
//                timeStamp = (Utilities.getUInt32(temp) / 16e6);
                for (int i = 0; i < Constants.AXIS; i++) {
                    for (int k = 0; k < Constants.BYTE_VAL; k++) {
                        temp[k] = buffer[j++];
                    }
                    inertialData[i] = ByteBuffer.wrap(temp).getFloat();    // x,y,z acc, gyro
                }
                chkSum = ((buffer[j] & 0xff) << 8) | (buffer[j + 1] & 0xFF);
//                System.out.println(cal_chksum(buffer)+ " and " +chkSum);
            } 
        }
    }

    public byte[] final_buffer(int len)
    {
        byte[] temp = new byte[Constants.DATA_LEN];
        try 
        {
            temp = read_data(len);
        }
        catch (StringIndexOutOfBoundsException e)
        {
            try 
            {
                Thread.sleep(5);
                temp = read_data(len);
            }
            catch (InterruptedException ex)
            {
                ex.printStackTrace();
                Thread.currentThread().interrupt();
            }
            e.printStackTrace();
        }
        return temp;
    }


    public void pharse_normal_old(byte[] buffer)
    {
        byte temp[] = new byte[4];
        int j = 0;
        if (buffer.length == Constants.DATA_LEN) 
        {
            for (int i = 0; i < 4; i++) 
            {
                header[i] = buffer[j++] & 0xFF;             //header define
            }
            startCode = header[0];                        // Receive AA
//            pktNum = (header[1] << 8) | header[2];
            pktNum = (header[1])*255 + header[2];
            payloadLen = header[3];                      // Payload Size

            if (startCode == 0xAA && payloadLen == 28) 
            {
                for (int i = 0; i < 4; i++) 
                {
                    temp[i] = buffer[j++];             //time_stamp define
                }

                for (int i = 0; i < Constants.AXIS; i++) 
                {
                    for (int k = 0; k < Constants.BYTE_VAL; k++) 
                    {
                        temp[k] = buffer[j++];
                    }
                    inertialData[i] = ByteBuffer.wrap(temp).getFloat();    // x,y,z acc, gyro
                }
                chkSum = ((buffer[j] & 0xff) << 8) | (buffer[j + 1] & 0xFF);
            }
        }
    }

    byte[] read_data(int len)
    {
        byte[] byt = new byte[1];
        len = len * 2;
        while ((ble_buffer.length() < (len*2)) && normal_imu)
        {
            String data_from_queue = mQueue.poll();
            if (data_from_queue != null) 
            {
                ble_buffer += data_from_queue;
//                System.out.println("data_from_queue : "+data_from_queue);
            }
            if (ble_buffer.length() > (len*2)) 
            {
                break;
            }
        }
//        System.out.println("data from queue normal: mqempty- "+ mQueue.isEmpty()+ " ble buffer length- " + ble_buffer.length() +" input size- "+len);
        if (ble_buffer.length() > len)
        {
            String take_str =  ble_buffer.substring(0,len);
            byt = Utilities.hexStringToByteArray(take_str);
            try 
            {
                ble_buffer = ble_buffer.substring(len, ble_buffer.length());
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }
        return byt;
    }

    public static int cal_chksum(byte[] data)
    {
        int checksum = 0;

        for (int i =0; i < data.length - 2; i++)
        {
            checksum += data[i] & 0xFF;
        }
        return checksum;
    }
    
}
