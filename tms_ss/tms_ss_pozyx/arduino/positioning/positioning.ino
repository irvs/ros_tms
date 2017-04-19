// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs
  
  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino
  
  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

int num_tags = 1;
uint16_t tags[1] = {0x6077};                        

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x680A, 0x6822, 0x683B, 0x685D};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {6580, 9775, 11050, 4831};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {6220, 5800, 100, 100};                  // anchor y-coordinates in mm
int32_t heights[4] = {2440, 2530, 2320, 2370};              // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_2D;                           // positioning dimension
int32_t height = 1300;                                  // height of device, required in 2.5D positioning

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  Pozyx.begin();
  
  setAnchorsManual();
  delay(1000);
}

void loop(){
  for (int i = 0; i < num_tags; i++){
    coordinates_t position;
//    euler_angles_t angles;
    quaternion_t q;
    int status = Pozyx.doRemotePositioning(tags[i], &position, dimension, height, algorithm);
//    Pozyx.getEulerAngles_deg(&angles,tags[i]);
    Pozyx.getQuaternion(&q,tags[i]);
    if (status == POZYX_SUCCESS){
      printCoordinates(position, q, tags[i]);
    }else{
      //printErrorCode("positioning", tags[i]);
      setAnchorsManual();
    }
  }
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor,quaternion_t q, uint16_t network_id){
  Serial.print(network_id);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  Serial.print(",");
  Serial.print(q.x);
  Serial.print(",");
  Serial.print(q.y);
  Serial.print(",");
  Serial.print(q.z);
  Serial.print(",");
  Serial.print(q.weight);
  Serial.print("\n");
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for (int i = 0; i < num_tags; i++){
    int status = Pozyx.clearDevices(tags[i]);
    for(int j = 0; j < num_anchors; j++){
      device_coordinates_t anchor;
      anchor.network_id = anchors[j];
      anchor.flag = 0x1; 
      anchor.pos.x = anchors_x[j];
      anchor.pos.y = anchors_y[j];
      anchor.pos.z = heights[j];
      status &= Pozyx.addDevice(anchor, tags[i]);
    }
  }
}
