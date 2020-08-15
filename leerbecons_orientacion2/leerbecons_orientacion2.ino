#include <stdlib.h>
#include <math.h>
#include <C:\Users\JOSE_L\Documents\Arduino\libraries\c_library_v2-master\common\mavlink.h>
#include <TinyGPS.h>

uint8_t _system_id = 254; // id of computer which is sending the command (ground control software has id of 255)

long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z, hedgehog_yaw;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received

bool high_resolution_mode;

#define HEDGEHOG_BUF_SIZE 40 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
unsigned int hedgehog_data_id;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

TinyGPS beacon1;
long lat1, lon1;
unsigned long age1;

 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint32_t time_week_ms=0; /*< [ms] GPS time (from start of GPS week)*/
 int32_t lat; /*< [degE7] Latitude (WGS84)*/
 int32_t lon; /*< [degE7] Longitude (WGS84)*/
 float alt; /*< [m] Altitude (MSL). Positive for up.*/
 float hdop1=1; /*< [m] GPS HDOP horizontal dilution of position*/
 float vdop=1; /*< [m] GPS VDOP vertical dilution of position*/
 float vn=0; /*< [m/s] GPS velocity in north direction in earth-fixed NED frame*/
 float ve=0; /*< [m/s] GPS velocity in east direction in earth-fixed NED frame*/
 float vd=0; /*< [m/s] GPS velocity in down direction in earth-fixed NED frame*/
 float speed_accuracy=0.1; /*< [m/s] GPS speed accuracy*/
 float horiz_accuracy=0.1; /*< [m] GPS horizontal accuracy*/
 float vert_accuracy=0.3; /*< [m] GPS vertical accuracy*/
 uint16_t ignore_flags=0; /*<  Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.*/
 uint16_t time_week=1; /*<  GPS week number*/
 uint8_t gps_id=3; /*<  ID of the GPS for multiple GPS inputs*/
 uint8_t fix_type=4; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK*/
 uint8_t satellites_visible=10; /*<  Number of satellites visible.*/
 uint16_t yaw; /*< [cdeg] Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north*/


void setup(){
     setup_hedgehog();//    Marvelmind hedgehog support initialize
     Serial.begin(500000);
     Serial1.begin(500000);
     Serial3.begin(500000);
}

void loop(){
   leerbeacon1();   
   loop_hedgehog();
   if (hedgehog_pos_updated){
       hedgehog_pos_updated= 0;
       if (high_resolution_mode){
        //resolucion alta mm
        //long hedgehog_x, hedgehog_y, hedgehog_z, hedgehog_yaw;
       }else{
        //resolucion baja cm
         //Serial.println(hedgehog_yaw);
         yaw=uint16_t(hedgehog_yaw*10);
         global_pos(); 
       }
   }
   
}

void leerbeacon1(){
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  for (unsigned long start = millis(); millis() - start < 10;){
    while (Serial3.available()){
      char c = Serial3.read();
      if (beacon1.encode(c))
        newData = true;
    }
  }

  if (newData){
    beacon1.get_position(&lat1, &lon1, &age1);
    float h=beacon1.f_altitude();
       lon=lat1*10;
       lat=lon1*10;
//      float velbeacon1.f_speed_mph();
       alt=h;
       global_pos();
  }  
}

//    Marvelmind hedgehog support initialize
static void setup_hedgehog() {
  Serial2.begin(500000); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;
}

// Marvelmind hedgehog service loop
static void loop_hedgehog(){
 int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 byte packet_size;
 uni_8x2_16 un16;
 uni_8x4_32 un32;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(Serial2.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial2.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte = 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte = 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);

              //orientacion
              un16.b[0]= hedgehog_serial_buf[17];
              un16.b[1]= hedgehog_serial_buf[18];
              hedgehog_yaw= un16.wi;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              hedgehog_z= un32.vi32;

              //orientacion
              un32.b[0]= hedgehog_serial_buf[23];
              un32.b[1]= hedgehog_serial_buf[24];
              un32.b[2]= 0;
              un32.b[3]= 0;
              hedgehog_yaw= un32.vi32 >> 20;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              break;
            }
          }
        } 
    }
}

// Calculate CRC-16 of hedgehog packet
static void hedgehog_set_crc16(byte *buf, byte size){
 uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}

void global_pos(){
 time_usec=micros();

 mavlink_message_t global_position;
 uint8_t buf_global_position[MAVLINK_MSG_ID_GPS_INPUT_LEN];
 mavlink_msg_gps_input_pack(_system_id,7,&global_position,time_usec,gps_id,ignore_flags,time_week_ms,time_week,fix_type,lat,lon,alt,hdop1,vdop,vn,ve,vd,speed_accuracy,horiz_accuracy,vert_accuracy,satellites_visible,yaw);

 uint16_t len = mavlink_msg_to_send_buffer(buf_global_position, &global_position);
 //Serial.write(buf_global_position, len);
 Serial1.write(buf_global_position, len);

}
