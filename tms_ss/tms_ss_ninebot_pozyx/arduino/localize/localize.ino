
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <math.h>

typedef struct{
  uint16_t id;
  double x;
  double y;
} anchor_t;

////////////////// PARAMETERS //////////////////

const uint8_t num_anchors = 4; // the number of anchors

anchor_t anchors[num_anchors] = {{0x6e08, -17.3741, -8.09985},
                                 {0x6e23, -6.13082, -4.69937},
                                 {0x6e58, -13.7048, -20.1612},
                                 {0x6e30, -2.47872, -16.6917}};

bool remote = true;
uint16_t remote_id = 0x6164;

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  Pozyx.begin();

  if(!remote) remote_id = NULL;

  AnchorsCalibration();
  
  delay(1000);
}

void loop(){

  coordinates_t position;
  quaternion_t orientation;

  int status;
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, POZYX_2D, 1000, POZYX_POS_ALG_UWB_ONLY);
  }else{
    status = Pozyx.doPositioning(&position, POZYX_2D, 1000, POZYX_POS_ALG_UWB_ONLY);
  }

  Pozyx.getQuaternion(&orientation, remote_id);
  
  if(status == POZYX_SUCCESS){
    printCoordinates(position, orientation, remote_id);
  }else{
    Serial.println(F("Positioning Error"));
    AnchorsCalibration();
  }
}

void printCoordinates(coordinates_t coor, quaternion_t quat, uint16_t remote_id){
  uint16_t network_id;
  if(remote){
    network_id = remote_id;
  }else{
    network_id = 0;
  }
  
  Serial.print(network_id);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print(",");
  Serial.print(quat.x);
  Serial.print(",");
  Serial.print(quat.y);
  Serial.print(",");
  Serial.print(quat.z);
  Serial.print(",");
  Serial.println(quat.weight);
}

void AnchorsCalibration(){
  Pozyx.clearDevices(remote_id);
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i].id;
    anchor.flag = 0x1;
    anchor.pos.x = (int32_t)round(anchors[i].x * 1000);
    anchor.pos.y = (int32_t)round(anchors[i].y * 1000);
    anchor.pos.z = 0;
    Pozyx.addDevice(anchor, remote_id);
  }
  if(num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  } 
}
