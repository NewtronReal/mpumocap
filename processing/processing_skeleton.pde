import processing.serial.*;

Serial port;
int no_of_imu = 1;
char[] sensorPacket = new char[14];
int interval=0;
int serialCount = 0;
int synced =0;

float q1[] =  new float[4];

void setup(){
  size(1920,1080,P3D);
  lights();
  smooth();
  //uncomment it to start serial port listening
  //String portName = Serial.list()[0];
  //port = new Serial(this,portName,115200);
}

float scale = 2;//scale of entire skeleton


//format {inital relative rotationsZ,Y,X,length of bone}
float rot1[] = {-PI/2,0,0,.4*scale*100};//chest(stationary)
float rot2[] = {PI/2,0,0,.5*scale*100};//hip
float rot3[] = {PI/6,0,0,.6*scale*100};//femour
float rot4[] = {-PI/6,0,0,.6*scale*100};//femour
float rot5[] = {-PI/6,0,0,.5*scale*100};//tibia
float rot6[] = {PI/6,0,0,.5*scale*100};//tibia
float rot[][] = {rot1,rot2,rot3,rot4,rot5,rot6};
float radius = 5*scale;//radius of bones
void draw(){
  background(200);
  //drawArm(200,200,rot1,rot2,new float[]{width/2,height/2,0});
  drawSkeleton(rot,new float[]{width/2,height/3,0});
}

void drawArm(float l1,float l2, float[] rot1,float[] rot2,float[] pos){
  translate(pos[0],pos[1],pos[2]);
  eulerRotate(rot1);
  translate(l1/2,0,0);
  box(l1,10,10);
  translate(l1/2,0,0);
  eulerRotate(rot2);
  translate(l2/2,0,0);
  box(l2,10,10);
}

void drawSkeleton(float[][] rot,float[] pos){
  translate(pos[0],pos[1],pos[2]);
  //chest
  eulerRotate(rot[0]);
  translate(rot[0][3]/2,0,0);
  box(rot[0][3],radius,radius);
  translate(-rot[0][3]/2,0,0);
  negEulerRotate(rot[0]);
  
  //hip
  eulerRotate(rot[1]);
  translate(rot[1][3]/2,0,0);
  box(rot[1][3],radius,radius);
  translate(rot[1][3]/2,0,0);
  //femour1
  eulerRotate(rot[2]);
  translate(rot[2][3]/2,0,0);
  box(rot[2][3],radius,radius);
  translate(rot[2][3]/2,0,0);
  //tibia1
  eulerRotate(rot[4]);
  translate(rot[4][3]/2,0,0);
  box(rot[4][3],radius,radius);
  
  //retrace to hip
  translate(-rot[4][3]/2,0,0);
  negEulerRotate(rot[4]);
  translate(-rot[2][3],0,0);
  negEulerRotate(rot[2]);
  
  //femour2
  eulerRotate(rot[3]);
  translate(rot[3][3]/2,0,0);
  box(rot[3][3],radius,radius);
  translate(rot[3][3]/2,0,0);
  //tibia2
  eulerRotate(rot[5]);
  translate(rot[5][3]/2,0,0);
  box(rot[5][3],radius,radius);
  
}

void eulerRotate(float rot[]){
  rotateZ(rot[0]);
  rotateY(rot[1]);
  rotateX(rot[2]);
}

void negEulerRotate(float rot[]){
  rotateZ(-rot[0]);
  rotateY(-rot[1]);
  rotateX(-rot[2]);
}

float deg(float degree){
  return degree*PI/180;
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
        rot[sensorPacket[1]-1][0] += atan2(2*q1[1]*q1[2] - 2*q1[0]*q1[3], 2*q1[0]*q1[0] + 2*q1[1]*q1[1] - 1);
        rot[sensorPacket[1]-1][1] += -asin(2*q1[1]*q1[3] + 2*q1[0]*q1[2]);
        rot[sensorPacket[1]-1][2] += atan2(2*q1[2]*q1[3] - 2*q1[0]*q1[1], 2*q1[0]*q1[0] + 2*q1[3]*q1[3] - 1);
      }
    }
  }
}
