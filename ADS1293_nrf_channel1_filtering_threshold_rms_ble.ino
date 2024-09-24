//this program does on-board filtering and detects thresholds, sends the signal wirelessly
//header files: SPI, Arduino, bluetooth comms, NRF headers for arduino

#include <SPI.h>
#include "Arduino.h"
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include "Adafruit_TinyUSB.h"
#include <InternalFileSystem.h>

//define each register as their names
#define WREG 0x7f
#define RREG 0x80

#define   CONFIG  0x00
#define   FLEX_CH1_CN   0x01
#define   FLEX_CH2_CN   0x02
#define   FLEX_CH3_CN   0x03
#define   FLEX_PACE_CN  0x04
#define   FLEX_VBAT_CN  0x05
#define   LOD_CN        0x06
#define   LOD_EN        0x07
#define   LOD_CURRENT   0x08
#define   LOD_AC_CN     0x09
#define   CMDET_EN      0x0a
#define   CMDET_CN      0x0b
#define   RLD_CN        0x0c


#define   REF_CN        0x11
#define   OSC_CN        0x12
#define   AFE_RES       0x13
#define   AFE_SHDN_CN   0x14
#define   AFE_FAULT_CN  0x15
#define   AFE_PACE_CN   0x17
#define   ERR_STATUS    0x19
#define   MASK_ERR      0x2a
#define   R2_RATE       0x21
#define   R3_RATE_CH1   0x22
#define   R3_RATE_CH2   0x23
#define   R3_RATE_CH3   0x24
#define   R1_RATE       0x25
#define   DIS_EFILTER   0x26
#define   DRDYB_SRC     0x27
#define   SYNCB_CN     0x28
#define   CH_CNFG       0x2f
#define   REVID         0x40

#define   POSITIVE_TST_SIG  0x01
#define   NEGATIVE_TST_SIG  0x02
#define   ZERO_TST_SIG  0x03
#define   ERROR         -1

//pin D0 for DRDY
//pin D2 for CSB
#define PIN_DRDY D0
#define PIN_CSB D2
#define rms_size 50 //size of each window
#define rms_factor 100 //number of windows in baseline
float threshold=0;//carries the value of threshold
float std_dev=0; //standard deviation of baseline
float avg=0; //average of baseline
float rms=0; //rms calculation
int phase=0; //state switch for device
int counter=0;//used to keep track of data collected for baseline
float baseline[5000];//baseline signal
float baseline_rms[rms_factor]; //make this baseline length/rms_size
int window_counter=0; //use to ensure we only do thresholding when this mark is reached
float window[rms_size]; //defined by rms_size
int on=0;

BLEService  emgReader = BLEService(0x181C); //emg reader service
BLECharacteristic emg = BLECharacteristic(0x2700); //unitless characteristic
BLEDis bledis; //device information service
//High pass filter designed as a class; can be instantiated as an object with an order of 1 or 2 using template
template <int order> // order is 1 or 2
class HighPass
{
  private:
    float a[order];//coefficient a
    float b[order+1];//there are order + 1 number of coefficient b
    float omega0;//2*pi*f = omega_0
    float dt;//delta time = 1/f
    bool adapt;//boolean value which makes filter adapt to a sampling frequency that keeps varying
    float tn1 = 0;// - delta time, made for easier reference
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    HighPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){//initialize all values of x and y to be 0 on first run
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();//used to initialize the values of the coefficients
    }

    void setCoef(){
      if(adapt){//if we have adapt turned on, calculate the frequency of data read by measuring time-stamp of each value with internal clock
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;// alpha is declared to make math easier to refer to
      if(order==1){
        float alphaFactor = 1/(1 + alpha/2.0);//alphfactor declared to make math simplified
        a[0] = -(alpha/2.0 - 1)*alphaFactor;//coefficients for a filter of order 1
        b[0] = alphaFactor;
        b[1] = -alphaFactor;      
      }
      if(order==2){
        float alpha = omega0*dt;
        float dtSq = dt*dt;
        float c[] = {omega0*omega0, sqrt(2)*omega0, 1};//c is declared as such to make repetitive math simplified in code
        float D = c[0]*dtSq + 2*c[1]*dt + 4*c[2];//denominator which recurrs among all coefficients
        b[0] = 4.0/D;//coefficient values for a filter of order 2
        b[1] = -8.0/D;
        b[2] = 4.0/D;
        a[0] = -(2*c[0]*dtSq - 8*c[2])/D;
        a[1] = -(c[0]*dtSq - 2*c[1]*dt + 4*c[2])/D;   
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;//last value of filtered value is initialized to 0
      x[0] = xn;//last value of raw value is initialized to input
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];//find the value of filtered using the following math: y0= a0*y1+a1*y2+ b0*x0+b1*x1+b2*x2 where y0 is the most recent value and the values go older in time
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){//shift all the values now, so the oldest value is dropped out of memory
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
HighPass<2> hp(20,533,0); //Highpass filter of 10Hz, sampling frequency of 533Hz, filter order of 2


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_DRDY, INPUT_PULLUP); //DRDY is our input to signal when data is ready
  pinMode(PIN_CSB,   OUTPUT);//CSB is output to singal ads1293 that we can start communication
  digitalWrite(PIN_CSB, HIGH);//CSB is active LOW, so default is HIGH

  
  Serial.begin(9600); 
  SPI.begin();
  
  Serial.println("starting");
  setup_EMG(); //execute function to initialize registers with relevant values
  phase=0;//indicator that we are currently getting baseline
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //Serial.println("LED turned on");
  
  Bluefruit.begin(); //Begin BLE setup
  Bluefruit.setName("REE");
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bledis.setManufacturer("Aeshyverse");
  bledis.setModel("NRF52840");
  bledis.begin();

  setupREE(); //sets up the BLE transmit routine
  startAdvertising(); //starts advertising BLE device for connection
}

void loop() {
  // put your main code here, to run repeatedly:
  

  
  if (digitalRead(PIN_DRDY) == false)
  { 
    //Serial.print("data received");
    int32_t ch1;
    float ych1;
    float n_ych1;
    ch1=getData(1);//single channel analysis, as of now
    //ch1=ch1/1e10;
    float conv_ch1=float(ch1); //conversion of long to floating point
    conv_ch1=(((conv_ch1/15925248)-0.5)*2*2.4/3.5); //conversion of output arduino value into voltage
    ych1=hp.filt(conv_ch1);  //butterworth filter, order 2 of input voltage
    n_ych1=ych1*1e6; //conversion for first 6 significant digits, for matlab processing
    long conv_ych1= long(n_ych1); //conversion to long: drop out the remaining digits
 //   transmitData(conv_ych1); //transmit to matlab

    if(phase==0) //thresholding part
    {
      counter++;
      if(counter%250==0)
      {
       digitalToggle(LED_BUILTIN); //blink the LED every 0.5s
      }
      if(counter>=2500&&counter<7500) //baseline collection starts 5s into start
      {
        baseline[counter-2500]=ych1;
      }
      if(counter>=7500) //after 15s have passed, stop baseline collections
      {
        phase=1; //state to signal thresholding
      }                          
    }
    if(phase==1)
    {
        digitalWrite(LED_BUILTIN, HIGH);//turn off LED in normal state
        //Serial.println("LED turned off");
        for(int k=0;k<rms_factor;k++) //find baseline rms with window size of 50
        {
          rms=0;
          for(int m=0;m<rms_size;m++)
          {
            int index=k*rms_size+m;
            rms=rms+baseline[index]*baseline[index];
          }
          baseline_rms[k]=sqrt(rms/rms_size);
        }
        for(int i=0;i<rms_factor;i++) //find average of baseline rms
        {
          avg=avg+baseline_rms[i];
        }
        avg=avg/rms_factor;
        for(int j=0;j<rms_factor;j++) //find std_dev of baseline rms
        {
          std_dev=std_dev+(baseline_rms[j]-avg)*(baseline_rms[j]-avg);
        }
        std_dev=sqrt(std_dev/rms_factor);
        threshold=avg+3*std_dev; //set threshold
        phase=2;
//        Serial.println("Average: ");
//        Serial.print(avg);
//        Serial.println("Stddev: ");
//        Serial.print(std_dev);
//        Serial.println("Rms: ");
//        Serial.print(rms);
        
    }
    if(phase==2)
    {
      window[window_counter]=ych1;
      window_counter++; //gather the values in each window
      if(window_counter==rms_size)
      {
        window_counter=0;
        rms=0;
        for(int i=0;i<rms_size;i++)
        {
          rms=rms+window[i]*window[i];
        }
        rms=sqrt(rms/rms_size);
      
        if(rms>(avg+3*std_dev))
        {
          digitalWrite(LED_BUILTIN, LOW);//turn on LED if detected
//        Serial.println("ON");
          on=1;
        }
        else
        {
          digitalWrite(LED_BUILTIN, HIGH);//keep LED off otherwise
//        Serial.println("OFF");
          on=0;
        }
      
//      Serial.println(ych1);
        int rms_int=rms*1e6; //the value to be displayed MUST be an integer so gather 6 sig digits
        emg.notify32(rms_int); //changes the transmitted data val
        transmitData(rms_int);  //send the data val now
        if ( Bluefruit.connected() ) {
         // Serial.println("Connected, sending notification");
     
          } 
        else {
       //   Serial.println("Not connected");
          }
      }
  }
}
}

void writeRegister(uint8_t wrAddress, uint8_t data)//subroutine to write values into ads1293 register
{
  uint8_t dataToSend = (wrAddress & WREG);//WREG is our terminator for written register, appended at the end
  digitalWrite(PIN_CSB, LOW);//give signal for communication to start
  
  SPI.transfer(dataToSend);
  SPI.transfer(data);
  digitalWrite(PIN_CSB, HIGH);
  
}

uint8_t readRegister(uint8_t rdAddress)//subroutine to read values from ads1293 register
{
  uint8_t rdData;
  uint8_t dataToSend = (rdAddress  | RREG);//RREG is our terminator for read register, appended at front
  digitalWrite(PIN_CSB, LOW);//give signal for communication to start
  //Serial.println("entered");
  SPI.transfer(dataToSend);
  rdData = SPI.transfer(0);
  digitalWrite(PIN_CSB, HIGH);

  return (rdData);

}

int32_t getData(uint8_t channel){//subroutine to read data for a specific channel

  uint8_t rawData[3];//8 bit array of 3 for a 24 bit data
  int32_t Data;

  if(channel < 1 || channel > 3){
    return -1;    //return error, -1
  }else {
    channel -= 1;//index for channel is shifted down by 1
  }

  rawData[0] = readRegister(0x37 + (channel * 3));//all the channel registers are arranged in sequence, so shift by 3 for other channels
  rawData[1] = readRegister(0x38 + (channel * 3));
  rawData[2] = readRegister(0x39 + (channel * 3));

  uint32_t tempData = (uint32_t)rawData[0]<<16;//the value read is 24 bit so arrange them in order in int32 datatype
  tempData = (uint32_t)rawData[1]<< 8;
  tempData |= rawData[2];
  tempData = tempData << 8;

  
  Data = (int32_t) (tempData); //conversion from unsigned to signed
  return (Data );
}

void transmitData(long adc)//subroutine to transfer a value via serial ports
{
  Serial.print(adc); //Serial transmission here
  Serial.write(13);
  Serial.write(10);
}

void setup_EMG()//subroutine to initialize the registers of the ads1293 with relevant values
{
  Serial.println("programmed ch1");
writeRegister(FLEX_CH1_CN,0x0A);//writing in channel 1 alone-> IN1 + IN2 -
//writeRegister(FLEX_CH2_CN,0x1C); //channel 2 -> IN3+ IN4-
//writeRegister(FLEX_CH3_CN,0x2E); //channel 3 -> IN5+ IN6-
delay(2);
Serial.println("programmed clock");
writeRegister(OSC_CN, 0x04);// clock -> ext crystal
//clock frequency used for all 3 channels: 102.4 KHz
delay(2);
Serial.println("programmed AFE");
writeRegister(AFE_RES , 0x07);// clock -> ext crystal
//enable high res mode
delay(2);
//writeRegister(AFE_RES , 0x07);// clock -> ext crystal
//enable high res mode
//delay(1);
//writeRegister(AFE_SHDN_CN,0x36); //channel 2&3 shutdown; remove for multiplexing
//delay(1);
Serial.println("programmed R2");
writeRegister(R2_RATE,0x04); // R2 divider = 6
delay(2);
Serial.println("programmed R3");
writeRegister(R3_RATE_CH1,0x04); //R3 divider for CH1=8
delay(2);
//writeRegister(R3_RATE_CH2,0x04); //R3 divider for CH2=8
//writeRegister(R3_RATE_CH3,0x04); //R3 divider for CH3=8
//R1_Rate of 4
//writeRegister(RLD_CN , 0x08);// Shutdown RLD
//delay(1);
Serial.println("programmed DRDY");
writeRegister(DRDYB_SRC , 0x08);// Set address 0x27 = 0x08: Configures the DRDYB source to channel 1 ECG (or fastest channel).
delay(2);
Serial.println("programmed CNFG");
writeRegister(CH_CNFG,0x10);//readback mode for CH1
delay(2);
Serial.println("started conversion");
writeRegister(00, 0x01);//start conversion
delay(2);
//writeRegister(CH_CNFG,0x70);//readback mode for CH1,2,3
//PACE channels, disconnected
//LEAD OFF check, disabled
// BAT check, disabled
//Common Mode, disabled
// RLD, disabled
// Wilson, disabled
// REF, disabled
//Error detections, disabled

}

void setupREE() { //function that sets up BLE routine
  emgReader.begin();

  // emg read characteristing occupies 3 bytes, but 4 bytes because of int
  

  emg.setProperties(CHR_PROPS_NOTIFY); //advertise whenever the property is changed
  emg.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); //security access
  emg.setFixedLen(4); //fix length of displayed value to 4 bytes - INT
  emg.begin();

}

  void startAdvertising() { //subroutine to advertise emgReader value
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(emgReader);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 80);   // in unit of 0.625 ms, change this for different frequencies of transmission
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}
/**
 * On connect
 */
void connectCallback(uint16_t conn_handle) {
}

/**
 * On disconnect
 */
void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
}
