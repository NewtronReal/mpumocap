import java.net.*;
import java.io.*;
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

DatagramSocket udpSocket;
InetAddress esp32Address;
int receivePort = 5555;         // Port to receive forwarded packets
int discoveryPort = 4444;       // Port to send discovery packet to ESP32
int[] dataArray = new int[14];  // Array to store received data

Serial port;
int no_of_imu = 2;
Quaternion quats[] = new Quaternion[no_of_imu];

float q1[] =  new float[4];
int box_length = 200;
int padding = height/2;

void setup() {
  for(int i=0;i<no_of_imu;i++){
    quats[i]=new Quaternion(1,0,0,0);
  }
  size(2048,1400,P3D);
  gfx = new ToxiclibsSupport(this);
  lights();
  smooth();
  try {
    // Set up UDP socket for receiving data
    udpSocket = new DatagramSocket(receivePort);
    udpSocket.setSoTimeout(10);  // Set a short timeout for non-blocking

    // Send discovery packet to ESP32
    esp32Address = InetAddress.getByName("vethramdevice.local");
    byte[] discoveryMessage = "DISCOVER".getBytes();
    DatagramPacket discoveryPacket = new DatagramPacket(discoveryMessage, discoveryMessage.length, esp32Address, discoveryPort);
    udpSocket.send(discoveryPacket);
    println("Discovery packet sent to ESP32 at " + esp32Address);
  } catch (Exception e) {
    e.printStackTrace();
    println("Error setting up UDP socket");
  }
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

void draw() {
  background(200);
  
  // Receive data from ESP32
  getData();

  boxdrawer();
}

void getData() {
  try {
    byte[] buffer = new byte[14];
    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

    // Attempt to receive packet (non-blocking)
    udpSocket.receive(packet);

    // Convert received bytes to unsigned integers
    for (int i = 0; i < packet.getLength(); i++) {
      dataArray[i] = (buffer[i] & 0xFF);  // Convert byte to unsigned int
    }
    q1[0] = ((dataArray[2] << 8) | dataArray[3]) / 16384.0f;
    q1[1] = ((dataArray[4] << 8) | dataArray[5]) / 16384.0f;
    q1[2] = ((dataArray[6] << 8) | dataArray[7]) / 16384.0f;
    q1[3] = ((dataArray[8] << 8) | dataArray[9]) / 16384.0f;
    for (int i = 0; i < 4; i++) if (q1[i] >= 2) q1[i] = -4 + q1[i];
    quats[dataArray[1]-1].set(q1[0], q1[1], q1[2], q1[3]);
  } catch (SocketTimeoutException e) {
    // No data received
  } catch (IOException e) {
    e.printStackTrace();
    println("Data retrieval error");
  }
}
