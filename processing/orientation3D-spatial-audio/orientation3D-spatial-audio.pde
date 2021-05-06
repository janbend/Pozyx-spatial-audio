import controlP5.*;
import oscP5.*;
import netP5.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.traces.*;
import org.gwoptics.graphics.graph2D.backgrounds.*;
import org.gwoptics.graphics.GWColour;
import processing.serial.*;
import java.lang.Math.*;
import java.text.DecimalFormat;

boolean serial = true;          // set to true to use Serial, false to use OSC messages.
boolean dragging = false;
int positionHistoryLength = 10;
int oscPort = 8888;               // change this to your UDP port
String serialPort = "COM8";      // change this to your COM port 
String ip_addr = "127.0.0.1";      // change the ipadress for osc messages


double room_x = 6200.0;      // change length of trackingarea in x-axis
double room_y = 2650.0;      // change length of trackingarea in y-axis

/////////////////////////////////////////////////////////////
//////////////////////  variables //////////////////////////
/////////////////////////////////////////////////////////////

ControlP5 cp5_imu_x;
ControlP5 cp5_imu_y;
ControlP5 cp5_imu_z;
ControlP5 cp5_imu_az;
ControlP5 cp5_imu_el;
ControlP5 cp5_i_or;
ControlP5 cp5_i_graph;
ControlP5 cp5_p_x;
ControlP5 cp5_p_y;
ControlP5 cp5_p_z;
ControlP5 cp5_dear_x;
ControlP5 cp5_dear_y;
ControlP5 cp5_dear_VR;
ControlP5 cp5_p_map;
ControlP5 cp5_controlpanel;
ControlP5 cp5_brightsign;
ControlP5 cp5_roomzone;
ControlP5 cp5_text;
ControlP5 cp5_calibration;
ControlP5 cp5_calibration_reset;
ControlP5 cp5_KFilter;

OscP5 oscP5;
Serial myPort;
NetAddress myRemoteLocation;

Toggle imu_x;
Toggle imu_y;
Toggle imu_z;
Toggle imu_az;
Toggle imu_el;
Toggle p_x;
Toggle p_y;
Toggle p_z;
Toggle dear_VR;
Toggle dear_x;
Toggle dear_y;
Toggle i_or;
Toggle i_graph;
Toggle p_map;
Toggle controlpanel;
Toggle brightsign;
Toggle roomzone;
Toggle calibration;
Toggle calibration_reset;

boolean toggleValue;
boolean zone1 = false;
boolean zone2 = false;
boolean zone3 = false;
boolean zone4 = false;
boolean zone5 = false;

int     lf = 10;       //ASCII linefeed
int[]   rgb_color = {229, 22, 110, 183, 0, 149, 60, 229, 46, 38};
String  inString;      //String for testing serial communication


Graph2D g_acc, g_gyro, g_mag;
PImage compass_img;

// initialization of kalman filter variables
double Rx = 4.0; // measurement noise covariance 
double Qx = 0.002;  // process noise covariance
double Ry = 5.; // measurement noise covariance 
double Qy = 0.0015;  // process noise covariance
double Rz = 100.0; // measurement noise covariance 
double Qz = 0.3;  // process noise covariance
double Kx;
double Ky;
double Kz;

double Xpe0 = 0.0;  // prior estimation of signal X at time t=0 (current state)
double Xe1 = 0.0;  // estimation of X at time t=1 (previous state)
double Pxpe0 = 0.0;  // prior estimation of "error covariance" at t=0  
double Px1 = 1.0; // error covariance at t=1
double Px0 = 0.0; // error covariance at t=0
double Xe0 = 0.0; // estimation of signal at t=0
double Xk = 0.0; // measured signal at t=0

double Ype0 = 0.0;  // prior estimation of signal Y at time t=0 (current state)
double Ye1 = 0.0;   // estimation of Y at time t=1 (previous state)
double Pype0 = 0.0; 
double Py1 = 1.0; 
double Py0 = 0.0; 
double Ye0 = 0.0;
double Yk = 0.0; 

double Zpe0 = 0.0;  // prior estimation of signal Z at time t=0 (current state)
double Ze1 = 0.0;  // estimation of Z at time t=1 (previous state)
double Pzpe0 = 0.0; 
double Pz1 = 1.0; 
double Pz0 = 0.0; 
double Ze0 = 0.0; 
double Zk = 0.0; 

int Xe0f = 1;
int Ye0f = 1;
int Ze0f = 1;

/////////////////////////////////////////////////////////////
///////////// sensordata variables //////////////////////////
/////////////////////////////////////////////////////////////

double pos_rx; // former int value, double when kalman filter
double pos_ry;
double pos_rz;
double pos_lrx;
double pos_lry;
double pos_lrz;

float x_angle = 0.0;  
float y_angle = 0.0;
float z_angle = 0.0;
float cx_angle = 0.0;  
float cy_angle = 0.0;
float cz_angle = 0.0;
float lx_angle = 0.0;  
float ly_angle = 0.0;
float lz_angle = 0.0;
float cquat_w= 0.0; 
float cquat_x= 0.0;
float cquat_y= 0.0;
float cquat_z= 0.0;
float z_angle_s = 0.0;
float z_angle_inv = 0.0;
float speed_x = 0.0;
float speed_y = 0.0;
float speed_z = 0;
float lin_acc_x = 0.0;
float lin_acc_y = 0.0;
float lin_acc_z = 0.0;
float quat_w, quat_x, quat_y, quat_z;
float grav_x, grav_y, grav_z;
float heading = 0.0;
float pressure = 0.0;

int   latency = 0;

String calib_status = "";
String calib_info = "uncalibrated";

Boolean calib_system = false;
Boolean sys_calibration = false;
Boolean res_calibration = false;

// array of sensor data over multiple timesteps
ArrayList<rangeData> accData;
ArrayList<rangeData> magData;
ArrayList<rangeData> gyroData;

// some variables for plotting the map
int border = 500;
int device_size = 15;
int offset_x = 30;
int offset_y = 30;
int thick_mark = 500;

float pixel_per_mm = 0.5;

// array for parsing anchor positions
int[] anchor_x = {0, 0, 0, 0};
int[] anchor_y = {0, 0, 0, 0};
int current_anchor = 0;

// creates an empty list of pozyx devices
PozyxDevice[] pozyxDevices = {}; 

class PozyxDevice {
  private int ID;
  private int[] pos_x = new int [positionHistoryLength];
  private int[] pos_y = new int [positionHistoryLength];
  private int[] pos_z = new int [positionHistoryLength];

  public PozyxDevice(int ID) {
    this.ID = ID;
  }

  public void addPosition(int x, int y, int z) {
    System.arraycopy(pos_x, 0, pos_x, 1, positionHistoryLength - 1);
    System.arraycopy(pos_y, 0, pos_y, 1, positionHistoryLength - 1);
    System.arraycopy(pos_z, 0, pos_z, 1, positionHistoryLength - 1);

    pos_x[0] = x;
    pos_y[0] = y;
    pos_z[0] = z;
  }

  public int[] getCurrentPosition() {
    int[] position ={pos_x[0], pos_y[0], pos_z[0]};
    return position;
  }

  public int[] getCurrentfilter() {
    int[] position ={Xe0f, Ye0f, Ze0f};
    return position;
  }
}

// creates a slider in decimalformat
public class CustomSlider extends Slider{

  //decimal format reference
  DecimalFormat df;

  public CustomSlider(ControlP5 cp5 , String name) {
    super(cp5,name);
    df = new DecimalFormat();
    df.setMaximumFractionDigits(2);
  }

  @Override public Slider setValue( float theValue ) {
    super.setValue(theValue);
    if(df != null){
      _myValueLabel.set( df.format(getValue( )));
    }else{
      _myValueLabel.set(getValue( ) + "");
    }
    return this;
  }
} 

/////////////////////////////////////////////////////////////
///////// class needed for the timeseries graph /////////////
/////////////////////////////////////////////////////////////

class rangeData implements ILine2DEquation {
  private double curVal = 0;

  public void setCurVal(double curVal) {
    this.curVal = curVal;
  }

  public double getCurVal() {
    return this.curVal;
  }

  public double computePoint(double x, int pos) {
    return curVal;
  }
}

void setup() {
  size(1100, 800, P3D);
  surface.setResizable(true);
  stroke(0, 0, 0);
  colorMode(RGB, 256); 

  // setup serial input
  if (serial) {
    try {
      myPort = new Serial(this, serialPort, 115200);
      myPort.clear();
      myPort.bufferUntil(lf);
    }
    catch(Exception e) {
      println("Cannot open serial port.");
    }
  } else {
    try {
      oscP5 = new OscP5(this, oscPort);
    }
    catch(Exception e) {
      println("Cannot open UDP port");
    }
  }

  // creates an empty list of pozyx devices
  oscP5 = new OscP5(this, 12000);   //listening
  myRemoteLocation = new NetAddress(ip_addr, 14000);  //  speak to

  // creates toggles for osc control
  cp5_imu_x = new ControlP5(this);
  cp5_imu_y = new ControlP5(this);
  cp5_imu_z = new ControlP5(this);
  cp5_imu_az = new ControlP5(this);
  cp5_imu_el = new ControlP5(this);
  cp5_p_x = new ControlP5(this);
  cp5_p_y = new ControlP5(this);
  cp5_p_z = new ControlP5(this);
  cp5_dear_VR = new ControlP5(this);
  cp5_dear_x = new ControlP5(this);
  cp5_dear_y = new ControlP5(this);
  cp5_i_or = new ControlP5(this);
  cp5_i_graph = new ControlP5(this);
  cp5_p_map = new ControlP5(this);
  cp5_controlpanel = new ControlP5(this);
  cp5_brightsign = new ControlP5(this);
  cp5_roomzone = new ControlP5(this);
  cp5_calibration = new ControlP5(this);
  cp5_calibration_reset = new ControlP5(this);
  cp5_text = new ControlP5(this);
  cp5_KFilter = new ControlP5(this);

  controlpanel = cp5_controlpanel.addToggle("")
    .setPosition(400, 60)
    .setValue(true)
    .setSize(10, 10);

  calibration = cp5_calibration.addToggle("")
    .setPosition(400, 80)
    .setValue(false)
    .setSize(10, 10);

  calibration_reset = cp5_calibration_reset.addToggle("")
    .setPosition(400, 100)
    .setValue(false)
    .setSize(10, 10);

  p_map = cp5_p_map.addToggle("")
    .setPosition(550, 60)
    .setValue(true)
    .setSize(10, 10);

  i_or = cp5_i_or.addToggle("")
    .setPosition(550, 80)
    .setValue(true)
    .setSize(10, 10);

  i_graph = cp5_i_graph.addToggle("")
    .setPosition(550, 100)
    .setValue(false)
    .setSize(10, 10);

  roomzone = cp5_roomzone.addToggle("")
    .setPosition(690, 60)
    .setValue(true)
    .setSize(10, 10);

  brightsign = cp5_brightsign.addToggle("")
    .setPosition(690, 80)
    .setValue(false)
    .setSize(10, 10);

  imu_az = cp5_imu_az.addToggle("")
    .setPosition(830, 60)
    .setValue(true)
    .setSize(10, 10);

  imu_el = cp5_imu_el.addToggle("")
    .setPosition(830, 80)
    .setValue(false)
    .setSize(10, 10);

  imu_x = cp5_imu_x.addToggle("")
    .setPosition(830, 100)
    .setValue(true)
    .setSize(10, 10);

  imu_y = cp5_imu_y.addToggle("")
    .setPosition(830, 120)
    .setValue(false)
    .setSize(10, 10);

  imu_z = cp5_imu_z.addToggle("")
    .setPosition(830, 140)
    .setValue(false)
    .setSize(10, 10);
    
  p_x = cp5_p_x.addToggle("")
    .setPosition(830, 160)
    .setValue(true)
    .setSize(10, 10);

  p_y = cp5_p_y.addToggle("")
    .setPosition(830, 180)
    .setValue(true)
    .setSize(10, 10);

  p_z = cp5_p_z.addToggle("")
    .setPosition(830, 200)
    .setValue(false)
    .setSize(10, 10);

  dear_VR = cp5_dear_VR.addToggle("")
    .setPosition(830, 220)
    .setValue(true)
    .setSize(10, 10);

  dear_x = cp5_dear_x.addToggle("")
    .setPosition(830, 240)
    .setValue(true)
    .setSize(10, 10);

  dear_y = cp5_dear_y.addToggle("")
    .setPosition(830, 260)
    .setValue(true)
    .setSize(10, 10);

  // initialize running traces 
  g_acc = new Graph2D(this, 400, 100, false);
  g_mag = new Graph2D(this, 400, 100, false);
  g_gyro = new Graph2D(this, 400, 100, false);   

  accData = new ArrayList<rangeData>();
  magData = new ArrayList<rangeData>();
  gyroData = new ArrayList<rangeData>();    
  for (int i=0; i<3; i++) {
    rangeData r = new rangeData();
    accData.add(r);
    RollingLine2DTrace rl = new RollingLine2DTrace(r, 100, 0.1f);
    rl.setTraceColour(rgb_color[i%10], rgb_color[(i+1)%10], rgb_color[(i+2)%10]);
    rl.setLineWidth(2);      
    g_acc.addTrace(rl);

    r = new rangeData();
    magData.add(r);
    rl = new RollingLine2DTrace(r, 100, 0.1f);
    rl.setTraceColour(rgb_color[i%10], rgb_color[(i+1)%10], rgb_color[(i+2)%10]);
    rl.setLineWidth(2);      
    g_mag.addTrace(rl);

    r = new rangeData();
    gyroData.add(r);
    rl = new RollingLine2DTrace(r, 100, 0.1f);
    rl.setTraceColour(rgb_color[i%10], rgb_color[(i+1)%10], rgb_color[(i+2)%10]);
    rl.setLineWidth(2);      
    g_gyro.addTrace(rl);
  }

  // create the accelerometer graph
  g_acc.setYAxisMin(-2.0f);
  g_acc.setYAxisMax(2.0f);
  g_acc.position.y = 50; 
  g_acc.setYAxisTickSpacing(0.5f);
  g_acc.setXAxisMax(5f);
  g_acc.setXAxisLabel("time (s)");
  g_acc.setYAxisLabel("acceleration [g]");
  g_acc.setBackground(new SolidColourBackground(new GWColour(1f, 1f, 1f)));

  // create the magnetometer graph
  g_mag.setYAxisMin(-80.0f);
  g_mag.setYAxisMax(80.0f);
  g_mag.position.y = 200;  
  g_mag.setYAxisTickSpacing(40f);
  g_mag.setXAxisMax(5f);
  g_mag.setXAxisLabel("time (s)");
  g_mag.setYAxisLabel("magnetic field strength [µT]");
  g_mag.setBackground(new SolidColourBackground(new GWColour(1f, 1f, 1f)));

  // create the gyrometer graph
  g_gyro.setYAxisMin(-1000.0f);
  g_gyro.setYAxisMax(1000.0f);
  g_gyro.position.y = 350;
  g_gyro.setYAxisTickSpacing(250f);
  g_gyro.setXAxisMax(5f);
  g_gyro.setXAxisLabel("time (s)");
  g_gyro.setYAxisLabel("angular velocity [deg/s]");
  g_gyro.setBackground(new SolidColourBackground(new GWColour(1f, 1f, 1f)));

  compass_img = loadImage("compass.png");
  
  cp5_text.addTextfield("IP-Address").setPosition(250, 100).setSize(100, 10).setAutoClear(false).setColorLabel(color(0,0,0)); ;
  cp5_text.addBang("Submit").setPosition(350, 100).setSize(10, 10).setColorLabel(color(0,0,0)).setColorForeground(color(220,220,220));
  
  // creates slider for kalman-filter gain
  CustomSlider slider = new CustomSlider(cp5_KFilter,"Noise1");
  slider.setPosition(50, 140).setSize(100, 10).setRange(0, 8).setValue(4);
  slider.setColorForeground(color(220,220,220));
  slider.setNumberOfTickMarks(11).setSliderMode(Slider.FLEXIBLE);
  slider.setCaptionLabel("Meas. Noise X").setColorLabel(color(0,0,0));
 
  CustomSlider slider2  = new CustomSlider(cp5_KFilter,"Noise2");
  slider2.setPosition(50, 160).setSize(100, 10).setRange(0.002, 0.02).setValue(0.002);
  slider2.setColorForeground(color(220,220,220));
  slider2.setNumberOfTickMarks(11).setSliderMode(Slider.FLEXIBLE);
  slider2.setCaptionLabel("Process. Noise X").setColorLabel(color(0,0,0));
  
  CustomSlider slider3 = new CustomSlider(cp5_KFilter,"Noise3");
  slider3.setPosition(50, 180).setSize(100, 10).setRange(0, 10).setValue(5);
  slider3.setColorForeground(color(220,220,220));
  slider3.setNumberOfTickMarks(11).setSliderMode(Slider.FLEXIBLE);
  slider3.setCaptionLabel("Meas. Noise Y").setColorLabel(color(0,0,0));
 
  CustomSlider slider4  = new CustomSlider(cp5_KFilter,"Noise4");
  slider4.setPosition(50, 200).setSize(100, 10).setRange(0.002, 0.02).setValue(0.002);
  slider4.setColorForeground(color(220,220,220));
  slider4.setNumberOfTickMarks(11).setSliderMode(Slider.FLEXIBLE);
  slider4.setCaptionLabel("Process. Noise Y").setColorLabel(color(0,0,0));
}

void draw() {
  background(220, 220, 220);
  fill(0, 0, 0);

  // Initialization of kalman variables
  double NoiseX_R = cp5_KFilter.getController("Noise1").getValue();
  double NoiseX_Q = cp5_KFilter.getController("Noise2").getValue();
  double NoiseY_R = cp5_KFilter.getController("Noise3").getValue();
  double NoiseY_Q = cp5_KFilter.getController("Noise4").getValue();

  // calculate discrete kalman filter
  Xk = pos_rx;
  Xpe0 = Xe1; //Assumption or prediction 1
  Pxpe0 = Px1 + NoiseX_Q; //Assumption or prediction 2
  Kx = Pxpe0/(Pxpe0 + NoiseX_R); // measurement update or correction of "Kalman gain"
  Xe0 = Xpe0 + Kx * (Xk - Xpe0); // measurement update or correction of "estimated signal"
  Px0 = (1 - Kx) * Pxpe0; // measurement update or correction of "error covariance"
  Xe1 = Xe0; // update: current t=0 becomes t=1 in the next step
  Px1 = Px0; // update: current t=0 becomes t=1 in the next step

  Yk = pos_ry;
  Ype0 = Ye1; // assumption or prediction 1
  Pype0 = Py1 + NoiseY_Q; // assumption or prediction 2
  Ky = Pype0/(Pype0 + NoiseY_R); // measurement update or correction of "Kalman gain"
  Ye0 = Ype0 + Ky * (Yk - Ype0); // measurement update or correction of "estimated signal"
  Py0 = (1 - Ky) * Pype0; // measurement update or correction of "error covariance"
  Ye1 = Ye0; // update: current t=0 becomes t=1 in the next step
  Py1 = Py0; // update: current t=0 becomes t=1 in the next step

  Zk = pos_rz;
  Zpe0 = Ze1; // assumption or prediction 1
  Pzpe0 = Pz1 + Qz; // assumption or prediction 2
  Kz = Pzpe0/(Pzpe0 + Rz); // measurement update or correction of "Kalman gain"
  Ze0 = Zpe0 + Kz * (Zk - Zpe0); // measurement update or correction of "estimated signal"
  Pz0 = (1 - Kz) * Pzpe0; // measurement update or correction of "error covariance"
  Ze1 = Ze0; // update: current t=0 becomes t=1 in the next step
  Pz1 = Pz0; // update: current t=0 becomes t=1 in the next step

  Xe0f = (int) Math.round(Xe0);
  Ye0f = (int) Math.round(Ye0);
  Ze0f = (int) Math.round(Ze0);

  // send osc message to control brightsign devices
  if (brightsign.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("start");  
    oscP5.send(newMessage, myRemoteLocation);
  } 

  // send serperate osc messages for orientation & position
  if (imu_az.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/imu_az/");  
    if (round(z_angle) > 180) {
      z_angle_s = (0.5+(1-(z_angle)/360));
    }
    if (round(z_angle) < 180) {
      z_angle_s = (0.5-((z_angle)/360));
    }
    newMessage.add(z_angle_s); 
    oscP5.send(newMessage, myRemoteLocation);
  } 

  if (imu_el.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/imu_el/");  
    newMessage.add((0.5+(y_angle/180))); 
    oscP5.send(newMessage, myRemoteLocation);
  } 

  if (imu_x.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/imu_x/");
    
    if (round(z_angle) < 180) {
      z_angle_s = (0.5 -((z_angle)/360));
    }
    if (round(z_angle) > 180) {
      z_angle_s = (0.5 +((360-z_angle)/360));
    }
    newMessage.add(z_angle_s);           
    oscP5.send(newMessage, myRemoteLocation);
    OscMessage nextMessage = new OscMessage("/imu_x_inv/");
    if(round(z_angle) < 180){
            z_angle_inv = 1-(z_angle/360);
    }else{z_angle_inv = (360-z_angle)/360;}
    nextMessage.add(z_angle_inv);
    oscP5.send(nextMessage, myRemoteLocation);
  } 

  if (imu_y.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/imu_y/");  
    newMessage.add(0.5+y_angle/180); 
    oscP5.send(newMessage, myRemoteLocation);
  } 

  if (imu_z.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/imu_z/");  
    newMessage.add((0.5+(x_angle/360))); 
    oscP5.send(newMessage, myRemoteLocation);
  } 

  // send osc messages for position    
  if (p_x.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/pos_x/");  
    float px = (float)((Xe0f+(15000-room_x/2))/30000.0);
    newMessage.add(px); 
    oscP5.send(newMessage, myRemoteLocation);
  }

  if (p_y.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/pos_y/");
    float py = (float)((Ye0f+(15000-room_y/2))/30000.0);
    newMessage.add(py); 
    oscP5.send(newMessage, myRemoteLocation);
  } 

  // send osc messages of to dearvr for cartesian mode
  if (dear_VR.getBooleanValue()) {
  float pz 
  = Xe0f; // scale distance in x-direction
  float px = Ye0f;
  float rz = pz;  
  float rx = px - Math.round(anchor_y[3]/2); 
  float r  = sqrt(Math.round(Math.pow(rz,2.0)+Math.pow(rx,2.0)));
  float sz = r*sin((z_angle+90.0)*PI/180);
  float sx = r*cos((z_angle+90.0)*PI/180);
  float offz = 0.5+(Math.round(room_x)/2)/60000;
  float offx = 0.5+(Math.round(room_y)/2)/60000;
  float dearz = offz + sz/60000;
  float dearx = offx + sx/60000;
  
  if (dear_x.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/dear_z/");
    newMessage.add(dearz); 
    oscP5.send(newMessage, myRemoteLocation);
  }
  
  if (dear_y.getBooleanValue()) {
    OscMessage newMessage = new OscMessage("/dear_x/");
    newMessage.add(dearx); 
    oscP5.send(newMessage, myRemoteLocation);
  }
  }
  
  // send osc messages of roomzone to digital audio workstation
  if (roomzone.getBooleanValue()) {
  int i = 0;    
  if(zone1 == true && i == 0){
  float volume1 = 0.5;
  float volume2 = 0.0;
  float volume3 = 0.0;
  float volume4 = 0.0;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new OscMessage("/zone4/");
  OscMessage newMessage5 = new OscMessage("/marker_zone1/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  oscP5.send(newMessage5, myRemoteLocation);
  i++;
  }
  else if(zone2 == true && i == 0){
  float volume1 = 0.0;
  float volume2 = 0.5;
  float volume3 = 0.0;
  float volume4 = 0.0;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new
  OscMessage("/zone4/");
  OscMessage newMessage5 = new OscMessage("/marker_zone2/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  oscP5.send(newMessage5, myRemoteLocation);
  i++;
  }
  else if(zone3 == true && i == 0){
  float volume1 = 0.0;
  float volume2 = 0.0;
  float volume3 = 0.5;
  float volume4 = 0.0;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new OscMessage("/zone4/");
  OscMessage newMessage5 = new OscMessage("/marker_zone3/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  oscP5.send(newMessage5, myRemoteLocation);
  i++;
  }
  else if(zone4 == true && i == 0){
  float volume1 = 0.0;
  float volume2 = 0.0;
  float volume3 = 0.0;
  float volume4 = 0.5;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new OscMessage("/zone4/");
  OscMessage newMessage5 = new OscMessage("/marker_zone4/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  oscP5.send(newMessage5, myRemoteLocation);
  i++;
  }
  else if(zone5 == true && i == 0){
  float volume1 = 0.0;
  float volume2 = 0.0;
  float volume3 = 0.0;
  float volume4 = 0.0;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new OscMessage("/zone4/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  i++;
  }else{
  if(i == 0){
  float volume1 = 0.5;
  float volume2 = 0.5;
  float volume3 = 0.5;
  float volume4 = 0.5;
  OscMessage newMessage1 = new OscMessage("/zone1/");
  OscMessage newMessage2 = new OscMessage("/zone2/");
  OscMessage newMessage3 = new OscMessage("/zone3/");
  OscMessage newMessage4 = new OscMessage("/zone4/");
  newMessage1.add(volume1);
  newMessage2.add(volume2);
  newMessage3.add(volume3);
  newMessage4.add(volume4);
  oscP5.send(newMessage1, myRemoteLocation);
  oscP5.send(newMessage2, myRemoteLocation);
  oscP5.send(newMessage3, myRemoteLocation);
  oscP5.send(newMessage4, myRemoteLocation);
  i++;
  }
  }
  }
  
  // draw the 3 timegraphs for IMU 
  if (i_graph.getBooleanValue()) {
    g_acc.position.x = (width - 450);        
    g_mag.position.x = (width - 450);
    g_gyro.position.x = (width - 450);    
    g_acc.draw();
    g_mag.draw();
    g_gyro.draw();
  } 

  // display localisation map 
  if (p_map.getBooleanValue()) {
    drawMap();
  }  

  //show 3D orientation data
  if (i_or.getBooleanValue()) {

    stroke(0, 0, 0);
    strokeWeight(0.01);

    pushMatrix();
    translate((width - 350), (height - 250), 150);

    rotateX(radians(-90));
    rotateZ(radians(90));
    quat_rotate(quat_w, quat_x, quat_y, quat_z);

    // draw the 3D box
    draw_rect(90, 110, 122);

    // draw lines
    strokeWeight(0.1);
    line(0, 0, 0, grav_x*2, grav_y*2, grav_z*2);    
    line(0, 0, 0, 0, 0, 1);

    // end rotation
    popMatrix();

    // draw the heading (compass)
    int img_size = 160;
    image(compass_img, (width - 150)-img_size/2, (height -120)-img_size/2, img_size, img_size);
    stroke(255, 0, 0);
    strokeWeight(3);
    line((width - 150), (height -120), (width - 150)+40*cos(radians(heading)), (height -120)+40*sin(radians(heading)));
  } 

   // show control
  if (controlpanel.getBooleanValue()) {
  show_gui();
  fill(90,110,122);
  text("SYSTEM", 195, 50);
  text("CONTROL", 437, 50);
  text("Reset Orientation", 420, 110);
  text("GUI", 600, 50);  
  text("Localisation Map", 570, 70);  
  text("IMU Orientation", 570, 90);
  text("IMU Graph", 570, 110);
  text("FUNCTIONS", 720, 50);
  text("Roomzone", 710, 70);
  text("Brightsign", 710, 90);
  text("Roomzone1: " + zone1, 710, 110);
  text("Roomzone2: " + zone2, 710, 130);
  text("Roomzone3: " + zone3, 710, 150);
  text("Roomzone4: " + zone4, 710, 170);
  text("Roomzone5: " + zone5, 710, 190);
  text("OSC", 880, 50);
  text("Azimuth", 850, 70);
  text("Elevation", 850, 90);
  text("Rotation X: " + z_angle + "°", 850, 110);
  text("Rotation Y: " + y_angle + "°", 850, 130);
  text("Rotation Z: " + x_angle + "°", 850, 150);
  text("Filter X: " + Xe0f + "mm", 850, 170);
  text("Filter Y: " + Ye0f +"mm", 850, 190);
  text("Filter Z: " + Ze0f + "mm", 850, 210);
  text("DearVR", 850, 230);
  text("DearVR Z", 850, 250);
  text("DearVR X", 850, 270);
  text("Meas. X: " + pos_rx + "mm", 50, 240);
  text("Dx " + (Math.round(Xe0-pos_rx)) + "mm", 180, 240);
  text("Meas. Y: " + pos_ry + "mm", 50, 260);
  text("Dy " + Math.round((Ye0-pos_ry)) + "mm", 180, 260);
  text("Meas. Z: " + pos_rz + "mm", 50, 280);
  text("Dz " + Math.round((Ze0-pos_rz)) + "mm", 180, 280);
  }else{
  hide_gui();
  }
  
  // text for system information
  fill(90,110,122);
  text("Ver. 1.0 - Build Date: "+ day() + "." + month() + "." + year(), 50, 20);
  text("(c) Pozyx Labs", width-200, 20);
  text("System: " + calib_info, 50, 70);
  text("Calibration Status:" + calib_status, 50, 90);
  text("IP:" + ip_addr, 150, 110);
  text("Latency: " + latency + "ms", 50, 110);
  text("Controlpanel", 420, 70);
  text("Calibrate Orientation", 420, 90);
}

void show_gui(){
  cp5_imu_x.show();
  cp5_imu_y.show();
  cp5_imu_z.show();
  cp5_imu_az.show();
  cp5_imu_el.show();
  cp5_p_x.show();
  cp5_p_y.show();
  cp5_p_z.show();
  cp5_dear_VR.show();
  cp5_dear_x.show();
  cp5_dear_y.show();
  cp5_i_or.show();
  cp5_i_graph.show();
  cp5_p_map.show();
  cp5_brightsign.show();
  cp5_roomzone.show();
  cp5_calibration_reset.show();
  cp5_text.show();
  cp5_KFilter.show();
  strokeWeight(1);
  stroke(90,110,122);
  fill(255,255,255);
  rect(40, 40,350,250);
  rect(390, 40,150,250);
  rect(540, 40,145,250);
  rect(685, 40,135,250);  
  rect(820, 40,150,250);
  fill(240,240,240);
  rect(40, 40,350,12);
  rect(390, 40,150,12);
  rect(540, 40,145,12);
  rect(820, 40,150,12);  
  rect(685, 40,135,12); 
}

void hide_gui(){
  cp5_imu_x.hide();
  cp5_imu_y.hide();
  cp5_imu_z.hide();
  cp5_imu_az.hide();
  cp5_imu_el.hide();
  cp5_p_x.hide();
  cp5_p_y.hide();
  cp5_p_z.hide();
  cp5_dear_VR.hide();
  cp5_dear_x.hide();
  cp5_dear_y.hide();
  cp5_i_or.hide();
  cp5_i_graph.hide();
  cp5_p_map.hide();
  cp5_brightsign.hide();
  cp5_roomzone.hide();
  cp5_calibration_reset.hide();
  cp5_text.hide();
  cp5_KFilter.hide(); 
}

// shape of orientation
void draw_rect(int r, int g, int b) {
  scale(60);
  beginShape(QUADS);
  fill(r, g, b);
  vertex(-1, 1.5, 0.25);
  vertex( 1, 1.5, 0.25);
  vertex( 1, -1.5, 0.25);
  vertex(-1, -1.5, 0.25);

  vertex( 1, 1.5, 0.25);
  vertex( 1, 1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5, 0.25);

  vertex( 1, 1.5, -0.25);
  vertex(-1, 1.5, -0.25);
  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);

  vertex(-1, 1.5, -0.25);
  vertex(-1, 1.5, 0.25);
  vertex(-1, -1.5, 0.25);
  vertex(-1, -1.5, -0.25);

  vertex(-1, 1.5, -0.25);
  vertex( 1, 1.5, -0.25);
  vertex( 1, 1.5, 0.25);
  vertex(-1, 1.5, 0.25);

  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5, 0.25);
  vertex(-1, -1.5, 0.25);

  endShape();
}

// math for quaternion
public void quat_rotate(float w, float x, float y, float z) {
  float _x, _y, _z;
  //if (q1.w > 1) q1.normalise(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
  double angle = 2 * Math.acos(w);
  float s = (float)Math.sqrt(1-w*w); // assuming quaternion normalised then w is less than 1, so term always positive.
  if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
    // if s close to zero then direction of axis not important
    _x = x; // if it is important that axis is normalised then replace with x=1; y=z=0;
    _y = y;
    _z = z;
  } else {
    _x = x / s; // normalise axis
    _y = y / s;
    _z = z / s;
  }
  rotate((float)angle, _x, _y, _z);
}

public final PVector quaternion_rotate(float w, float x, float y, float z, PVector v) { 
  float q00 = 2.0f * x * x;
  float q11 = 2.0f * y * y;
  float q22 = 2.0f * z * z;

  float q01 = 2.0f * x * y;
  float q02 = 2.0f * x * z;
  float q03 = 2.0f * x * w;

  float q12 = 2.0f * y * z;
  float q13 = 2.0f * y * w;

  float q23 = 2.0f * z * w;

  return new PVector((1.0f - q11 - q22) * v.x + (q01 - q23) * v.y
    + (q02 + q13) * v.z, (q01 + q23) * v.x + (1.0f - q22 - q00) * v.y
    + (q12 - q03) * v.z, (q02 - q13) * v.x + (q12 + q03) * v.y
    + (1.0f - q11 - q00) * v.z);
}

// draw localisation map
void drawMap() {
  int plane_width =  width - 2 * offset_x;
  int plane_height = height - 2 * offset_y;
  
  // draw the plane
  stroke(0);
  fill(255);
  rect(offset_x, offset_y, plane_width, plane_height);
  calculateAspectRatio();
  pushMatrix(); 
  
  // translate grid
  translate(offset_x + (border * pixel_per_mm), height - offset_y - (border * pixel_per_mm));
  rotateX(radians(180));

  // draw the grid
  strokeWeight(1);
  stroke(100);
  
  for (int i = 0; i < (int) plane_width/pixel_per_mm/thick_mark; i++)
    line(i * thick_mark * pixel_per_mm, - thick_mark * pixel_per_mm, i * thick_mark * pixel_per_mm, plane_height - thick_mark * pixel_per_mm);
    
  stroke(100);
  
  for (int i = 0; i < (int) plane_height/pixel_per_mm/thick_mark - 1; i++)
    line(-(thick_mark * pixel_per_mm), i * thick_mark * pixel_per_mm, plane_width-(thick_mark * pixel_per_mm), (i* thick_mark * pixel_per_mm));

  if (roomzone.getBooleanValue()) {
  border();

  if(Xe0f < 0 || Ye0f < 0){
  zone1 = false;  
  zone2 = false;
  zone3 = false;
  zone4 = false;  
  zone5 = true;
  }else if(Xe0f < anchor_x[3]/2 && Ye0f < anchor_y[3]/2){
  zone1 = true;
  zone2 = false;
  zone3 = false;
  zone4 = false;  
  zone5 = false;
  }else if(Xe0f < anchor_x[3]/2 && Ye0f > anchor_y[3]/2){
  zone1 = false;
  zone2 = true; 
  zone3 = false;
  zone4 = false;  
  zone5 = false;
  }else if(Xe0f > anchor_x[3]/2 && Ye0f < anchor_y[3]/2){
  zone1 = false;
  zone2 = false;
  zone3 = true;
  zone4 = false;  
  zone5 = false;
  }else if(Xe0f > anchor_x[3]/2 && Ye0f > anchor_y[3]/2){
  zone1 = false;
  zone2 = false;
  zone3 = false;
  zone4 = true;
  zone5 = false;
  }
  } 
  
  drawDevices();
  stroke(0);
  drawArrow(0, 0, 50, 0.);
  drawArrow(0, 0, 50, 90.);
  pushMatrix();
  stroke(120, 142, 156);
  line(anchor_x[0]*pixel_per_mm,anchor_y[0]*pixel_per_mm,anchor_x[1]*pixel_per_mm,anchor_y[0]*pixel_per_mm);
  line(anchor_x[0]*pixel_per_mm,anchor_y[3]*pixel_per_mm,anchor_x[1]*pixel_per_mm,anchor_y[3]*pixel_per_mm);
  line(anchor_x[0]*pixel_per_mm,anchor_y[0]*pixel_per_mm,anchor_x[0]*pixel_per_mm,anchor_y[2]*pixel_per_mm);
  line(anchor_x[1]*pixel_per_mm,anchor_y[0]*pixel_per_mm,anchor_x[1]*pixel_per_mm,anchor_y[2]*pixel_per_mm);
  rotateX(radians(180));
  text("X", 55, 5);
  text("Y", -3, -55);
  text(anchor_x[3]+"mm", anchor_x[3]*pixel_per_mm/2, 12);
  rotate(HALF_PI);
  text(anchor_y[3]+"mm", -anchor_y[3]*pixel_per_mm/2, 12);
  popMatrix();  
  popMatrix();
}

// add devices in localisation map
void addPosition(int ID, int x, int y, int z) {
  for (PozyxDevice pozyxDevice : pozyxDevices) {
    // ID in device list already
    if (pozyxDevice.ID == ID) {
      pozyxDevice.addPosition(x, y, z);
      return;
    }
  }
  // pozyx id not in device list
  PozyxDevice newPozyx = new PozyxDevice(ID);
  newPozyx.addPosition(x, y, z);
  pozyxDevices = (PozyxDevice[]) append(pozyxDevices, newPozyx);
}

void drawDevices() {
  for (PozyxDevice pozyxDevice : pozyxDevices) {
    drawDevice(pozyxDevice);
  }
}

// size of localisation map
void calculateAspectRatio() {
  float plane_width =  width - 2 * offset_x;
  float plane_height = height - 2 * offset_y;
  int max_width_mm = 0;
  int max_height_mm = 0;
  for (PozyxDevice pozyxDevice : pozyxDevices) {
    int[] pos = pozyxDevice.getCurrentPosition();
    max_width_mm = max(max_width_mm, pos[0]);
    max_height_mm = max(max_height_mm, pos[1]);
  }
  max_width_mm += 2*border; 
  max_height_mm += 2*border; 
  pixel_per_mm = min(pixel_per_mm, plane_width / max_width_mm, plane_height / max_height_mm);
}

// devices in localisation map
void drawDevice(PozyxDevice device) {  
  stroke(0, 0, 0);
  fill(0, 0, 0);
  ellipseMode(CORNER);
  int[] current_position = device.getCurrentPosition();
  ellipse(pixel_per_mm * current_position[0] - device_size/2, pixel_per_mm * current_position[1] - device_size/2, device_size, device_size);
  fill(120, 145, 156);
  int[] current_filter = device.getCurrentfilter();
  ellipse(pixel_per_mm * current_filter[0] - device_size/2, pixel_per_mm * current_filter[1] - device_size/2, device_size, device_size);
  pushMatrix();
  rotateX(radians(180));
  fill(0);
  textSize(11);
  text("0x" + hex(device.ID, 4), pixel_per_mm * current_position[0] - 3 * device_size / 2, - pixel_per_mm * current_position[1] + device_size);
  textSize(12);
  popMatrix();
}  

// draw borders dynamically
void border() {
  for (PozyxDevice pozyxDevice : pozyxDevices) {
    border(pozyxDevice);
  }
}
void border(PozyxDevice device) {  
    stroke(120, 142, 156);
    strokeWeight(2);
    int[] current_position = device.getCurrentPosition(); 
    ellipseMode(CORNER);
    ellipse(pixel_per_mm * current_position[0] - device_size/2, pixel_per_mm * current_position[1] - device_size/2, device_size, device_size);
    line(anchor_x[1]*pixel_per_mm/2,anchor_y[0]*pixel_per_mm,anchor_x[1]*pixel_per_mm/2,anchor_y[2]*pixel_per_mm);
    line(anchor_x[0]*pixel_per_mm,anchor_y[3]*pixel_per_mm/2,anchor_x[1]*pixel_per_mm,anchor_y[2]*pixel_per_mm/2);
}

// device x-, y-axis in localisation map
void drawArrow(int center_x, int center_y, int len, float angle) {
  pushMatrix();
  translate(center_x, center_y);
  rotate(radians(angle));
  strokeWeight(2);
  line(0, 0, len, 0);
  line(len, 0, len - 8, -8);
  line(len, 0, len - 8, 8);
  popMatrix();
}

// read and buffer serial input
void serialEvent(Serial p) {
  inString = (myPort.readString());
  println(inString);  
  try {
    // parse the data
    String[] dataStrings = split(inString, ',');

    if (dataStrings[0].equals("ANCHOR")) {
      int id = Integer.parseInt(split(dataStrings[1], 'x')[1], 16);
      addPosition(id, int(dataStrings[2]), int(dataStrings[3]), int(dataStrings[4]));
      anchor_x[current_anchor] = int(dataStrings[2]);
      anchor_y[current_anchor] = int(dataStrings[3]);
      current_anchor++;
    }

    // check for serial input localisation tracking
    if (dataStrings[31].equals("POS") || dataStrings[0].equals("ANCHOR")) {
      int id = Integer.parseInt(split(dataStrings[32], 'x')[1], 16);
      addPosition(id, int(dataStrings[33]), int(dataStrings[34]), int(dataStrings[35]));
      pos_rx= int(dataStrings[33]); 
      pos_ry= int(dataStrings[34]); 
      pos_rz= int(dataStrings[35]);

      // buffer datastream if zero
      if (pos_rx == 0) {
        pos_rx = pos_lrx;
      } else {
        pos_lrx = pos_rx;
      }

      if (pos_ry == 0) {
        pos_ry = pos_lry;
      } else {
        pos_lry = pos_ry;
      }

      if (pos_rz == 0) {
        pos_rz = pos_lrz;
      } else {
        pos_lrz = pos_rz;
      }
    }

    // check for latency or interrupts of measurement
    latency = int(dataStrings[0]);

    // the pressure from mPa to Pa is coming in at a slower rate
    pressure = float(dataStrings[3])/1000.0f;   

    // acceleration from mg to g
    accData.get(0).setCurVal(float(dataStrings[4])/1000.0f);      
    accData.get(1).setCurVal(float(dataStrings[5])/1000.0f);
    accData.get(2).setCurVal(float(dataStrings[6])/1000.0f);

    // magnetometer data in µT
    magData.get(0).setCurVal(float(dataStrings[7])/16.0f);      
    magData.get(1).setCurVal(float(dataStrings[8])/16.0f);
    magData.get(2).setCurVal(float(dataStrings[9])/16.0f);

    // gyroscope data in degrees per second
    gyroData.get(0).setCurVal(float(dataStrings[10])/16.0f);      
    gyroData.get(1).setCurVal(float(dataStrings[11])/16.0f);
    gyroData.get(2).setCurVal(float(dataStrings[12])/16.0f);

    quat_w = float(dataStrings[16])/16384.0f;
    quat_x = float(dataStrings[17])/16384.0f;
    quat_y = float(dataStrings[18])/16384.0f;
    quat_z = float(dataStrings[19])/16384.0f;
    float norm = PApplet.sqrt(quat_x * quat_x + quat_y * quat_y + quat_z
    * quat_z +quat_w * quat_w);
    quat_w = quat_w/norm;
    quat_x = quat_x/norm;
    quat_y = quat_y/norm;
    quat_z = quat_z/norm;  
    println(norm);

    // linear acceleration in mg    
    lin_acc_x = float(dataStrings[20]);
    lin_acc_y = float(dataStrings[21]);
    lin_acc_z = float(dataStrings[22]);

    // gravitation vector from mg to g
    grav_x = float(dataStrings[23])/16000.0f;
    grav_y = float(dataStrings[24])/16000.0f; 
    grav_z = float(dataStrings[25])/16000.0f;

    // the calibration status
    calib_status = "Mag: " + dataStrings[27] + " - Acc: " + dataStrings[28] + " - Gyro: " + dataStrings[29] + " - System: " + dataStrings[30];
    int mag_result = Integer.parseInt(dataStrings[27]);
    int acc_result = Integer.parseInt(dataStrings[28]);
    int sys_result = Integer.parseInt(dataStrings[30]);
      
     if (calibration.getBooleanValue()) {
     sys_calibration = true;
     }else{sys_calibration = false;}
     
     if (calibration_reset.getBooleanValue()) {
     res_calibration = true;
     calib_system = false;
     sys_calibration = false;
     }else{res_calibration = false;}

      if((mag_result >= 2 && calib_system == false)){
      calib_info = "wait for calibration";
      }
      if((mag_result == 3 && acc_result == 3 && sys_result == 3 && calib_system == false)){
      cx_angle = float(dataStrings[15])/16.0f;
      cy_angle = float(dataStrings[14])/16.0f;
      cz_angle = float(dataStrings[13])/16.0f;
      calib_info = "ready for calibration";
      }else{
      x_angle = float(dataStrings[15])/16.0f;
      y_angle = float(dataStrings[14])/16.0f;
      z_angle = float(dataStrings[13])/16.0f;
      heading = z_angle;
      }
    
     if(sys_calibration == true || calib_system == true ){
      x_angle = (float(dataStrings[15])/16.0f - cx_angle) % 180;
      y_angle = (float(dataStrings[14])/16.0f - cy_angle) % 180;
      z_angle = (float(dataStrings[13])/16.0f - cz_angle + 360) % 360;
      heading = z_angle;
      calib_system = true;
      calib_info = "calibrated";
     }
  }
  catch (Exception e) {
    println("Error while reading serial data.");
  }
}

void Submit() {
  ip_addr=cp5_text.get(Textfield.class, "IP-Address").getText();
}


// comment out - recieve osc messages via network

/*void oscEvent(OscMessage theOscMessage) {
 // osc message received
 println("### received an osc message with addrpattern "+theOscMessage.addrPattern()+" and typetag "+theOscMessage.typetag());
 if (theOscMessage.addrPattern().equals("/sensordata")){
 //theOscMessage.print();}
 try{
 // the pressure from mPa to Pa is coming in at a slower rate
 pressure = theOscMessage.get(1).floatValue();
 
 // acceleration from mg to g
 accData.get(0).setCurVal(theOscMessage.get(2).floatValue()/1000.0f);      
 accData.get(1).setCurVal(theOscMessage.get(3).floatValue()/1000.0f);
 accData.get(2).setCurVal(theOscMessage.get(4).floatValue()/1000.0f);
 
 // magnetometer data in µT
 magData.get(0).setCurVal(theOscMessage.get(5).floatValue());      
 magData.get(1).setCurVal(theOscMessage.get(6).floatValue());
 magData.get(2).setCurVal(theOscMessage.get(7).floatValue());
 
 // gyroscope data in degrees per second
 gyroData.get(0).setCurVal(theOscMessage.get(8).floatValue());      
 gyroData.get(1).setCurVal(theOscMessage.get(9).floatValue());
 gyroData.get(2).setCurVal(theOscMessage.get(10).floatValue());
 
 // Euler angles in degrees    
 x_angle = theOscMessage.get(13).floatValue();
 y_angle = theOscMessage.get(12).floatValue();
 z_angle = theOscMessage.get(11).floatValue();
 heading = theOscMessage.get(11).floatValue();
 
 // the orientation quaternion
 quat_w = theOscMessage.get(14).floatValue();
 quat_x = theOscMessage.get(15).floatValue();
 quat_y = theOscMessage.get(16).floatValue();
 quat_z = theOscMessage.get(17).floatValue();
 float norm = PApplet.sqrt(quat_x * quat_x + quat_y * quat_y + quat_z
 * quat_z +quat_w * quat_w);     
 quat_w = quat_w/norm;
 quat_x = quat_x/norm;
 quat_y = quat_y/norm;
 quat_z = quat_z/norm;  
 println(norm);
 
 // linear acceleration in mg    
 lin_acc_x = theOscMessage.get(18).floatValue();
 lin_acc_y = theOscMessage.get(19).floatValue();
 lin_acc_z = theOscMessage.get(20).floatValue();
 
 // gravitation vector from mg to g
 grav_x = theOscMessage.get(21).floatValue()/1000.0f;
 grav_y = theOscMessage.get(22).floatValue()/1000.0f; 
 grav_z = theOscMessage.get(23).floatValue()/1000.0f;
 
 // the calibration status
 calib_status = "Mag: " + str(theOscMessage.get(24).intValue()) + " - Acc: " + str(theOscMessage.get(25).intValue()) + " - Gyro: " + str(theOscMessage.get(26).intValue()) + " - System: " + str(theOscMessage.get(27).intValue());
 }catch(Exception e){
 println("Error while receiving OSC sensor data");
 }
 }
 }*/
