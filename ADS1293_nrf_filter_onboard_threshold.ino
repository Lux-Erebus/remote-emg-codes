#include <SPI.h>
#include "Arduino.h"


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
double EMA_a=0.1;
int32_t EMA_S=0;
double baseline_avg[j];
double bavg;
double baseline_std=0;
int counter=0;
uint32_t baseline[5000];
void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_DRDY, INPUT_PULLUP); //might be INPUT
  pinMode(PIN_CSB,   OUTPUT);
  digitalWrite(PIN_CSB, HIGH);

  
  Serial.begin(9600); 
  SPI.begin();
  
  Serial.println("starting");
  setup_EMG();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  

  
  if (digitalRead(PIN_DRDY) == false)
  { 
    //Serial.print("data received");
    int32_t ch1;
    ch1=getData(1);//single channel analysis, as of now
    double ch1dat=(ch1/15925248.0-0.5)*(2.0*2.4/3.5);     //0xF30000 is ADCmax at 204.8 kHz with R1=4,R2=6,R3=16  
    double ch1dat2=ch1dat*1000000;
    int32_t rounded_ch1=ch1dat2;
    
    uint32_t filtered_ch1=highpassfilter(rounded_ch1);
    if(counter<2500)
    {
      counter=counter+1;
      Serial.println("waiting");
    }
    
    else if(counter<7500)
    {
      baseline[counter-2500]=filtered_ch1;
      counter=counter+1;
      Serial.println("sampling");
    }
    else if(counter==7500)
    {
      
      counter=8000;
      for (int j=0;j<10;j++) //finding average of baseline
      {
        for(int i=j*500; i<(j+1)*500; i++)
        {
          baseline_avg[j]=baseline_avg[j]+baseline[i];
        }
        baseline_avg[j]=baseline_avg[j]/500;
      }
      for(int k=0;k<10;k++)//normalization across all values
      {
        bavg=bavg+baseline_avg[k]/10;
      }

      for(int i=0;i<5000;i++) //finding standard deviation
      {
        baseline_std=baseline_std+ ((double)baseline[i]-bavg)*((double)baseline[i]-bavg)/5000
      }
      baseline_std=sqrt(baseline_std);
    }
    
    if(counter==8000)
    {
      //Serial.println("Threshold: ");
      //Serial.println(threshold);
      if((filtered_ch1>(baseline_avg+3*baseline_std))||(filtered_ch1<(baseline_avg-3*baseline_std)))
      Serial.println("active");
      else
      Serial.println("inactive");
    }
  }
  
}

void writeRegister(uint8_t wrAddress, uint8_t data)
{
  uint8_t dataToSend = (wrAddress & WREG);
  digitalWrite(PIN_CSB, LOW);
  
  SPI.transfer(dataToSend);
  SPI.transfer(data);
  digitalWrite(PIN_CSB, HIGH);
  
}

uint8_t readRegister(uint8_t rdAddress)
{
  uint8_t rdData;
  uint8_t dataToSend = (rdAddress  | RREG);
  digitalWrite(PIN_CSB, LOW);
  //Serial.println("entered");
  SPI.transfer(dataToSend);
  rdData = SPI.transfer(0);
  digitalWrite(PIN_CSB, HIGH);

  return (rdData);

}

int32_t getData(uint8_t channel){

  uint8_t rawData[3];
  int32_t Data;

  if(channel < 1 || channel > 3){
    return -1;    //return error, -1
  }else {
    channel -= 1;
  }

  rawData[0] = readRegister(0x37 + (channel * 3));
  rawData[1] = readRegister(0x38 + (channel * 3));
  rawData[2] = readRegister(0x39 + (channel * 3));

  uint32_t tempData = (uint32_t)rawData[0]<<16;
  tempData = (uint32_t)rawData[1]<< 8;
  tempData |= rawData[2];
  tempData = tempData << 8;

  
  Data = (int32_t) (tempData); 
  return (Data );
}

void transmitData(uint32_t adc)
{
  Serial.print(adc); //Serial transmission here
  Serial.write(13);
  Serial.write(10);
}

void setup_EMG()
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

uint32_t highpassfilter(uint32_t input)
{
  EMA_S=(EMA_a*input)+((1-EMA_a)*EMA_S);
  uint32_t highpass=input-EMA_S;
  return highpass;
}
