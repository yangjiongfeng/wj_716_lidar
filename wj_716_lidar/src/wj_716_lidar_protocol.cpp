#include "wj_716_lidar_protocol.h"
#include <iostream>

namespace wj_lidar
{
  bool wj_716_lidar_protocol::setConfig(wj_716_lidar::wj_716_lidarConfig &new_config,uint32_t level)
  {
    config_ = new_config;
    scan.header.frame_id = config_.frame_id;
    scan.angle_min = config_.min_ang;
    scan.angle_max = config_.max_ang;
    scan.angle_increment = config_.angle_increment;
    scan.time_increment = config_.time_increment;//0.00002469;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;
//    scan.ranges.resize(config_.resize);
    scan.intensities.resize(config_.resize);

    scan_TwoEcho.header.frame_id = config_.frame_id;
    scan_TwoEcho.angle_min = config_.min_ang;
    scan_TwoEcho.angle_max = config_.max_ang;
    scan_TwoEcho.angle_increment = config_.angle_increment;
    scan_TwoEcho.time_increment = config_.time_increment;//0.00002469;
    scan_TwoEcho.range_min = config_.range_min;
    scan_TwoEcho.range_max = config_.range_max;
//    scan.ranges.resize(config_.resize);
    scan.intensities.resize(config_.resize);

    cout << "frame_id:" <<scan.header.frame_id<<endl;
    cout << "min_ang:" <<scan.angle_min<<endl;
    cout << "max_ang:" <<scan.angle_max<<endl;
    cout << "angle_increment:" <<scan.angle_increment<<endl;
    cout << "time_increment:" <<scan.time_increment<<endl;
    cout << "range_min:" <<scan.range_min<<endl;
    cout << "range_max:" <<scan.range_max<<endl;
    int resizeNum;
    cout << "resizeNum:" <<resizeNum<<endl;
    return true;
  }
   wj_716_lidar_protocol::wj_716_lidar_protocol()
   {
     memset(&m_sdata,0,sizeof(m_sdata));
     marker_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
     twoecho_=nh.advertise<sensor_msgs::LaserScan>("scan_TwoEcho", 5);
     ros::Time scan_time = ros::Time::now();      //make a virtual data per sec
     scan.header.stamp = scan_time;

     g_u32PreFrameNo = 0;

     scan.header.frame_id = "wj_716_lidar_frame";
     scan.angle_min = -2.35619449;
     scan.angle_max = 2.35619449;
     scan.angle_increment = 0.00582;
     scan.time_increment = 0.0667/1081;
     scan.range_min = 0;
     scan.range_max = 30;
     scan.ranges.resize(811);
     scan.intensities.resize(811);

     scan_TwoEcho.header.frame_id = "wj_716_lidar_frame";
     scan_TwoEcho.angle_min = -2.35619449;
     scan_TwoEcho.angle_max = 2.35619449;
     scan_TwoEcho.angle_increment = 0.00582;
     scan_TwoEcho.time_increment = 0.0667/1081;
     scan_TwoEcho.range_min = 0;
     scan_TwoEcho.range_max = 50;
     scan_TwoEcho.ranges.resize(811);
     scan_TwoEcho.intensities.resize(811);
     cout << "wj_716_lidar_protocl start success" << endl;

   }
   bool wj_716_lidar_protocol::dataProcess(const char *data, const int reclen)
   {
     if(reclen > MAX_LENGTH_DATA_PROCESS)
     {
       return false;
     }

     if(m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
       return false;
     }
     memcpy(&m_sdata.m_acdata[m_sdata.m_u32in],data,reclen*sizeof(char));
     m_sdata.m_u32in += reclen;
     while(m_sdata.m_u32out < m_sdata.m_u32in)
     {
       if(m_sdata.m_acdata[m_sdata.m_u32out] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out+2] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+3] == 0x02)
       {
         unsigned l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                                 (m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                                 (m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8)  |
                                 (m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
         l_u32reallen = l_u32reallen + 9;

         if(l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
         {
           if(OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out],l_u32reallen))
           {
             m_sdata.m_u32out += l_u32reallen;
           }
           else
           {
             cout << "continuous frame"<<endl;
             int i;
             for(i == 1; i<l_u32reallen; i++)
             {
               if((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 2] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 3] == 0x02))
               {
                 m_sdata.m_u32out += i;
                 break;
               }
               if(i == l_u32reallen)
               {
                 m_sdata.m_u32out += l_u32reallen;
               }
             }
           }
         }
         else if(l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
         {
           cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS"<<endl;
           cout << "reallen: "<<l_u32reallen<<endl;
           memset(&m_sdata,0,sizeof(m_sdata));

         }
         else
         {
           //cout<<"reallen: "<<l_u32reallen<<" indata: "<<m_sdata.m_u32in<<" outdata: "<<m_sdata.m_u32out<<endl;
           break;
         }
       }
       else
       {
         m_sdata.m_u32out++;
       }
     } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

     if(m_sdata.m_u32out >= m_sdata.m_u32in)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
     }
      return true;
   }

   bool wj_716_lidar_protocol::OnRecvProcess( char *data, int len)
   {
     if(len > 0)
     {
       if(checkXor(data,len))
       {
         protocl(data,len);
       }
     }
     else
     {
       return false;
     }
     return true;
   }

   bool wj_716_lidar_protocol::protocl(const char *data, const int len)
   {
     if((data[8] == 0x73 && data[9] == 0x52)||(data[8] == 0x73 && data[9] == 0x53) )   //command type:0x73 0x52/0X53
     {
       static int s_n32ScanIndex;
       int l_n32PackageNo= (data[50] << 8) + data[51];                                        //shuju bao xu hao
       unsigned int l_u32FrameNo = (data[46]<<24) + (data[47]<<16) + (data[48]<<8) + data[49];        //quan hao

       if(l_n32PackageNo == 1)
       {
         s_n32ScanIndex = 0;
         g_u32PreFrameNo = l_u32FrameNo;

         for(int j = 0; j < 810;j=j+2)
         {
           scandata[s_n32ScanIndex] = (((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
           scandata[s_n32ScanIndex] /= 1000.0;
           if(scandata[s_n32ScanIndex]>=25 || scandata[s_n32ScanIndex]==0)
           {
             scandata[s_n32ScanIndex]= NAN;
           }
           s_n32ScanIndex++;
         }
          //cout << "quan hao1: " << g_u32PreFrameNo << "  danzhen quanhao1: " << l_u32FrameNo<< endl;
       }
       else if(l_n32PackageNo == 2)
       {
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           for(int j = 0; j < 812;j=j+2)
           {
             scandata[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             scandata[s_n32ScanIndex] /= 1000.0;
             if(scandata[s_n32ScanIndex]>=25 || scandata[s_n32ScanIndex]==0)
             {
               scandata[s_n32ScanIndex]= NAN;
             }
             scan.intensities[s_n32ScanIndex] = 0;
             s_n32ScanIndex++;
           }

         }
         else
         {
           s_n32ScanIndex = 0;
           //cout << "quan hao2: " << g_u32PreFrameNo << "  danzhen quanhao2: " << l_u32FrameNo<< endl;
           return false;
         }
       }
       else if(l_n32PackageNo == 3)
       {
         s_n32ScanIndex=0;
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           for(int j = 0; j < 810;j=j+2)
           {
             scaninden[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             s_n32ScanIndex++;
           }
         }
         else
         {
           s_n32ScanIndex = 0;
           //cout << "quan hao3: " << g_u32PreFrameNo << "  danzhen quanhao3: " << l_u32FrameNo<< endl;
           return false;
         }
       }
       else if(l_n32PackageNo == 4)
       {
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           for(int j = 0; j < 812;j=j+2)
           {
             scaninden[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             s_n32ScanIndex++;
           }

           // adjust angle_min to min_ang config param
           int index_min = (config_.min_ang+2.35619449)/scan.angle_increment;
           // adjust angle_max to max_ang config param
           int index_max =  811-((2.35619449-config_.max_ang)/ scan.angle_increment);
           scan.ranges.resize(index_max-index_min);
           scan.intensities.resize(index_max-index_min);

           for (int j = index_min; j <= index_max; ++j)
           {
               scan.ranges[j - index_min] = scandata[j];
               scan.intensities[j - index_min]=scaninden[j];
           }
         }
         else
         {
           s_n32ScanIndex = 0;
           //cout << "quan hao4: " << g_u32PreFrameNo << "  danzhen quanhao4: " << l_u32FrameNo<< endl;
           return false;
         }


       }
       else if(l_n32PackageNo == 5)
       {
         s_n32ScanIndex=0;
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           for(int j = 0; j < 810;j=j+2)
           {
             scandata_te[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             scandata_te[s_n32ScanIndex] /= 1000.0;
             if(scandata_te[s_n32ScanIndex]>=55 || scandata_te[s_n32ScanIndex]==0)
             {
               scandata_te[s_n32ScanIndex]= NAN;
             }
             s_n32ScanIndex++;
           }
         }
         else
         {
           s_n32ScanIndex = 0;
           //cout << "quan hao5: " << g_u32PreFrameNo << "  danzhen quanhao5: " << l_u32FrameNo<< endl;
           return false;
         }
       }
       else if(l_n32PackageNo == 6)
       {
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           for(int j = 0; j < 812;j=j+2)
           {
             scandata_te[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             scandata_te[s_n32ScanIndex] /= 1000.0;
             if(scandata_te[s_n32ScanIndex]>=55 || scandata_te[s_n32ScanIndex]==0)
             {
               scandata_te[s_n32ScanIndex]= NAN;
             }
             s_n32ScanIndex++;
           }

           int index_min = (config_.min_ang+2.35619449)/scan_TwoEcho.angle_increment;
           // adjust angle_max to max_ang config param
           int index_max =  811-((2.35619449-config_.max_ang)/ scan_TwoEcho.angle_increment);
           scan_TwoEcho.ranges.resize(index_max-index_min);
           scan_TwoEcho.intensities.resize(index_max-index_min);

           for (int j = index_min; j <= index_max; ++j)
           {
               scan_TwoEcho.ranges[j - index_min] = scandata_te[j];
               scan_TwoEcho.intensities[j - index_min] =0;
           }

           ros::Time scan_time = ros::Time::now();
           scan.header.stamp = scan_time;
           marker_pub.publish(scan);

           ros::Time scan_time1 = ros::Time::now();      //make a virtual data per sec
           scan_TwoEcho.header.stamp = scan_time1;
           twoecho_.publish(scan_TwoEcho);
           //cout<<"time_increament_ "<<scan.time_increment<<endl;

         }
         else
         {
           s_n32ScanIndex = 0;
           //cout << "quan hao6: " << g_u32PreFrameNo << "  danzhen quanhao6: " << l_u32FrameNo<< endl;
           return false;
         }



       }
       return true;
     }
     else
     {
       return false;
     }

   }

   bool wj_716_lidar_protocol::checkXor( char *recvbuf,  int recvlen)
   {
     int i = 0;
     char check = 0;
     char *p = recvbuf;
     int len;
     if(*p == (char)0x02)
     {
       p = p+8;
       len = recvlen - 9;
       for(i = 0; i < len; i++)
       {
         check ^= *p++;
       }
       if(check == *p)
       {
         return true;
       }
       else
         return false;
     }
     else
     {
       return false;
     }
   }

}
