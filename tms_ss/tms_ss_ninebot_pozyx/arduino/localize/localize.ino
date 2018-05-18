
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

const uint8_t num_anchors = 8; // the number of anchors

// anchor_t anchors[num_anchors] = {{0x6e23, 10.073, -9.460},
//                                  {0x6e49, 11.540, 8.810},
//                                  {0x6e08, -7.235, 6.019},
//                                  {0x6e22, -8.224, -10.837},
//                                  {0x6e31, 1.598, -30.148},
//                                  {0x6e58, -2.504, -52.789},
//                                  {0x6e39, -7.115, -41.871},
//                                  {0x6e30, -6.999, -35.079}};

anchor_t anchors[num_anchors] = {{0x6e23, 10.073, -9.460},
                                 {0x6e49, 11.540, 8.810},
                                 {0x6e08, -7.235, 6.019},
                                 {0x6e22, -3.422, -18.685},
                                 {0x6e31, 1.598, -30.148},
                                 {0x6e58, -2.504, -52.789},
                                 {0x6e39, -7.115, -41.871},
                                 {0x6e30, -6.999, -35.079}};

bool remote = true;
uint16_t remote_id = 0x6e28;

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;
uint8_t dimension = POZYX_2D;
int32_t height = 1000;

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
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
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
  uint16_t network_id = 0;
  if(remote) network_id = remote_id;
  
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
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;

//    double offset_x = 12.838;
//    double offset_y = 3.330;

    anchor.network_id = anchors[i].id;
    anchor.flag = 0x1;
    anchor.pos.x = (int32_t)round(anchors[i].x * 1000);
    anchor.pos.y = (int32_t)round(anchors[i].y * 1000);

//    anchor.pos.x = (int32_t)round((anchors[i].x - offset_x) * 1000);
//    anchor.pos.y = (int32_t)round(-(anchors[i].y - offset_y) * 1000);
    anchor.pos.z = 0;
    Pozyx.addDevice(anchor, remote_id);
  }
  if(num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  } 
}
