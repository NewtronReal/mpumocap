import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

int[][] colors = {
    {255, 0, 0},      // Red
    {0, 255, 0},      // Green
    {0, 0, 255},      // Blue
    {255, 255, 0},    // Yellow
    {255, 0, 255},    // Magenta
    {0, 255, 255},    // Cyan
    {128, 0, 0},      // Dark Red
    {0, 128, 0},      // Dark Green
    {0, 0, 128},      // Dark Blue
    {128, 128, 0},    // Olive
    {128, 0, 128},    // Purple
    {0, 128, 128},    // Teal
    {192, 192, 192},  // Silver
    {128, 128, 128},  // Gray
    {255, 165, 0},    // Orange
    {75, 0, 130}      // Indigo
};

Serial port;
int no_of_imu = 2;
char[] sensorPacket = new char[14];
Quaternion quats[] = new Quaternion[no_of_imu];
int interval=0;
int serialCount = 0;
int synced =0;
float[] euler = new float[4];

float q1[] =  new float[4];
int box_length = 200;
int padding = height/2;
void setup(){
  for(int i=0;i<no_of_imu;i++){
    quats[i]=new Quaternion(1,0,0,0);
  }
  size(2048,1400,P3D);
  gfx = new ToxiclibsSupport(this);
  lights();
  smooth();
  String portName = Serial.list()[0];
  port = new Serial(this,portName,115200);
}


void boxdrawer(){
  int no_cols = floor(sqrt(quats.length));
  float margin_scale = 2;
  float trans_height = height/2-(no_cols*margin_scale*box_length)/2;
  float trans_width = width/2-(no_cols*margin_scale*box_length)/2;
  
  for(int i =0;i<quats.length;i++){
    pushMatrix();
    translate(trans_width+box_length*((i)/no_cols)*margin_scale,trans_height+box_length*((i)%no_cols)*margin_scale);
    fill(colors[i][0],colors[i][1],colors[i][2]);
    float[] axis = quats[i].toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);
    box(box_length);
    stroke(0);
    popMatrix();
  }
}

void draw(){
  background(0);
  boxdrawer();
}

void serialEvent(Serial port){
  interval = millis();
  while(port.available()>0){
    int ch = port.read();
    if(synced == 0 && ch!='$'){return;};
    synced = 1;
    if((serialCount == 1 && ch<1) || (serialCount == 12 && ch!='\r') || (serialCount == 13 && ch !='\n')){
      serialCount = 0;
      synced=0;
      return;
    }
    if(serialCount>0 || ch=='$'){
      sensorPacket[serialCount++] = (char)ch;
      if(serialCount == 14){
        serialCount=0;
        
        q1[0] = ((sensorPacket[2] << 8) | sensorPacket[3]) / 16384.0f;
        q1[1] = ((sensorPacket[4] << 8) | sensorPacket[5]) / 16384.0f;
        q1[2] = ((sensorPacket[6] << 8) | sensorPacket[7]) / 16384.0f;
        q1[3] = ((sensorPacket[8] << 8) | sensorPacket[9]) / 16384.0f;
        for (int i = 0; i < 4; i++) if (q1[i] >= 2) q1[i] = -4 + q1[i];
        quats[sensorPacket[1]-1].set(q1[0], q1[1], q1[2], q1[3]);
      }
    }
  }
}
