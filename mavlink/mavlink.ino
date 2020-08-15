//#include <mavlink.h>
#include <C:\Users\JOSE_L\Documents\Arduino\libraries\c_library_v2-master\common\mavlink.h>
  uint8_t _system_id = 254; // id of computer which is sending the command (ground control software has id of 255)

void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output
 
request_datastream();
 
}
 
void loop() {
 
MavLink_receive();
//mocap(); 
//gps();
//delay(100);
//attitud();
//vision();
delay(100);
}

void MavLink_receive(){ 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial1.available())
  {
    uint8_t c= Serial1.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
 
        case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&msg, &packet);
        
        Serial.print("\nGPS Fix: ");Serial.println(packet.fix_type);
        Serial.print("GPS Latitude: ");Serial.println(packet.lat);
        Serial.print("GPS Longitude: ");Serial.println(packet.lon);
        Serial.print("GPS Speed: ");Serial.println(packet.vel);
        Serial.print("Sats Visible: ");Serial.println(packet.satellites_visible);
       
      }
      break;

      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t packet_o;
        mavlink_msg_attitude_decode(&msg, &packet_o);
        
        Serial.print("yaw ");Serial.println(packet_o.roll);
        Serial.print("pitch ");Serial.println(packet_o.pitch);
        Serial.print("roll ");Serial.println(packet_o.yaw);
       
      }
      break;
      default:
      //Serial.print("GPS Speed: ");
      break;
      }
    }
  }
}
 
void request_datastream() {
//Request Data from Pixhawk
  uint8_t _system_id = 254; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  Serial1.write(buf, len); //Write data to serial port
}

void mocap(){
  uint8_t _component_id = 30; // seems like it can be any # except the number of what Pixhawk sys_id is

 float q[4]={1,0,0,0};
 float x=0;
 float y=0;
 float z=0; 
 float covariance[21];
 uint16_t tim=micros();


 mavlink_message_t mocapmsg;
 uint8_t buf_mocap[MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN];
 mavlink_msg_att_pos_mocap_pack(_system_id,_component_id,&mocapmsg,tim,q,x,y,z,covariance);
 uint16_t len = mavlink_msg_to_send_buffer(buf_mocap, &mocapmsg);
 Serial1.write(buf_mocap, len); //Write data to serial port

}

void gps(){
 uint8_t _component_id = 36; // seems like it can be any # except the number of what Pixhawk sys_id is
 
 uint64_t time_usec=micros(); /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint32_t time_week_ms=0; /*< [ms] GPS time (from start of GPS week)*/
 int32_t lat=90; /*< [degE7] Latitude (WGS84)*/
 int32_t lon=90; /*< [degE7] Longitude (WGS84)*/
 float alt=90; /*< [m] Altitude (MSL). Positive for up.*/
 float hdop=1; /*< [m] GPS HDOP horizontal dilution of position*/
 float vdop=1; /*< [m] GPS VDOP vertical dilution of position*/
 float vn=9; /*< [m/s] GPS velocity in north direction in earth-fixed NED frame*/
 float ve=8; /*< [m/s] GPS velocity in east direction in earth-fixed NED frame*/
 float vd=8; /*< [m/s] GPS velocity in down direction in earth-fixed NED frame*/
 float speed_accuracy=1; /*< [m/s] GPS speed accuracy*/
 float horiz_accuracy=1; /*< [m] GPS horizontal accuracy*/
 float vert_accuracy=1; /*< [m] GPS vertical accuracy*/
 uint16_t ignore_flags=809; /*<  Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.*/
 uint16_t time_week=3; /*<  GPS week number*/
 uint8_t gps_id=1; /*<  ID of the GPS for multiple GPS inputs*/
 uint8_t fix_type=2; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK*/
 uint8_t satellites_visible=10; /*<  Number of satellites visible.*/
 uint16_t yaw=90;

 mavlink_message_t gpsmsg;
 uint8_t buf_gps[MAVLINK_MSG_ID_GPS_INPUT_LEN];
 mavlink_msg_gps_input_pack(254,4,&gpsmsg,time_usec,gps_id,ignore_flags,time_week_ms,time_week,fix_type,lat,lon,alt,hdop,vdop,vn,ve,vd,speed_accuracy,horiz_accuracy,vert_accuracy,satellites_visible,yaw);
 uint16_t len = mavlink_msg_to_send_buffer(buf_gps, &gpsmsg);
 Serial1.write(buf_gps, len);
}

void attitud(){
 uint32_t time_boot_ms=micros(); /*< [ms] Timestamp (time since system boot).*/
 float roll=0; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch=0; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw=0; /*< [rad] Yaw angle (-pi..+pi)*/
 float rollspeed=0; /*< [rad/s] Roll angular speed*/
 float pitchspeed=0; /*< [rad/s] Pitch angular speed*/
 float yawspeed=0;

 mavlink_message_t attitudmsg;
 uint8_t buf_attitud[MAVLINK_MSG_ID_ATTITUDE_LEN];
 mavlink_msg_attitude_pack(_system_id,4,&attitudmsg,time_boot_ms,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed);
 uint16_t len = mavlink_msg_to_send_buffer(buf_attitud, &attitudmsg);
 Serial1.write(buf_attitud, len);
 
}

void vision(){
 uint64_t usec=0; /*< [us] Timestamp (UNIX time or time since system boot)*/
 float x=1; /*< [m] Local X position*/
 float y=0; /*< [m] Local Y position*/
 float z=9; /*< [m] Local Z position*/
 float roll=1; /*< [rad] Roll angle*/
 float pitch=9; /*< [rad] Pitch angle*/
 float yaw=8; /*< [rad] Yaw angle*/
 float covariance[21]; /*<  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t reset_counter=30;
 
 mavlink_message_t visionmsg;
 uint8_t buf_vision[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
 mavlink_msg_vision_position_estimate_pack(_system_id,5,&visionmsg,usec,x,y,z,roll,pitch,yaw,covariance,reset_counter);
 uint16_t len = mavlink_msg_to_send_buffer(buf_vision, &visionmsg);
 Serial1.write(buf_vision, len);
}
