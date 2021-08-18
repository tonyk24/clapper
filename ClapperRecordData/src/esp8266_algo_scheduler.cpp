#include <Arduino.h>
#include <stdio.h>
#include <iostream>
#include <list>
#include <mutex>
#include <climits>

#include "libmn.hpp"
#include "mn_mutex.hpp"


// Sample rate in Hz
#define SAMPLE_RATE 2000
// Recording time in milliseconds
#define TIME 200

// Overlap in percent, TODO: can be in long_term_steps
// Overlap in milliseconds
#define OVERLAP 40

#define OVERLAP_NUM (int)((float)OVERLAP / 1000.0 * SAMPLE_RATE)
#define SAMPLES_NUM_OVERLAP (OVERLAP_NUM + SAMPLE_RATE * TIME / (1000))
#define SAMPLES_NUM (int)(SAMPLE_RATE * TIME / (1000)) 

// Time we measure the window in
#define LIMIT_MEASURE_TIME_US (1000000 / SAMPLE_RATE)

// Buffer to store the samples in
float samplesB[2][SAMPLES_NUM_OVERLAP];
// Keep track of which buffer we are in, 0 or 1
int bufferIndex = 1;

std::mutex bufferMutex;
//basic_mutex bufferMutex;

#define LED_RED_PIN 10
#define LED_BLUE_PIN 5

using namespace std;

void turnOnRedLed() {
  digitalWrite(LED_RED_PIN, HIGH);
}

void turnOffRedLed() {
  digitalWrite(LED_RED_PIN, LOW);
}


void turnOnBlueLed() {
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void turnOffBlueLed() {
  digitalWrite(LED_BLUE_PIN, LOW);
}

class ClapDataContainer
{
  public:
  ClapDataContainer() {}
  ClapDataContainer(const ClapDataContainer &rhs) {
    this->ts = rhs.ts;
    for (int i=0; i<m_BUF_SIZE; i++) {
      this->buffer[i] = rhs.buffer[i];
      yield();
    }
    this->trueOrFalse = rhs.trueOrFalse;
    yield();
  }
  static const int m_BUF_SIZE = 75;
  unsigned long ts;
  float buffer[m_BUF_SIZE];
  bool trueOrFalse;
};

class ReadingTask : public basic_thread {
public:
    ReadingTask() : basic_thread("ReadingTask", 1) { }

    virtual void*  on_thread() override { 
        basic_thread::on_thread(); 
        
        while(true) {
          //readingValues();
        }

        return NULL; 
    }

     
};

class PrintTask : public basic_thread {
public:
    PrintTask() : basic_thread("PrintTask", 1,USHRT_MAX) { }

    float algo1_short_term_s = 0.0025;
    float algo1_long_term_mul = 15;
    int algo1_short_term_steps = algo1_short_term_s * SAMPLE_RATE;
    int algo1_long_term_steps = algo1_long_term_mul * algo1_short_term_steps;

    int algo1_during_clap = 0;


    const double algo1_decision_threshold = 12.0;
    int counter = 0;  

    virtual void*  on_thread() override { 
        basic_thread::on_thread(); 
        
        while(true) 
        {
            Serial.print(counter);
            Serial.println("************************************************************************");
            bufferMutex.lock();
            Serial.print("InternalBufferIndex= ");
            Serial.println((bufferIndex-1)%2);
            bufferMutex.unlock();

            counter++;
            list<ClapDataContainer> ret;  
            double short_term_average_sum = 0.0;
            double long_term_average_sum = 0.0;
            for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
              int internalBufferIndex = 0;
              
              bufferMutex.lock();
              internalBufferIndex = bufferIndex-1;
              bufferMutex.unlock();

              yield();
              if (i >= algo1_long_term_steps) {
                // Calculate long term average
                long_term_average_sum = 0.0;
                for (int j=i-algo1_long_term_steps; j<i; j++) {
                  bufferMutex.lock();
                  long_term_average_sum += samplesB[internalBufferIndex%2][j];
                  bufferMutex.unlock();
                }
          
                double long_term_average = long_term_average_sum / (double)algo1_long_term_steps;
          
                short_term_average_sum = 0.0;
                // Calculate short term average
                for (int j=i-algo1_short_term_steps; j<i; j++) {
                  bufferMutex.lock();
                  short_term_average_sum += samplesB[internalBufferIndex%2][j];
                  bufferMutex.unlock();
                }
          
                double short_term_average = short_term_average_sum / (double)algo1_short_term_steps;
                ClapDataContainer cdc;
                if (long_term_average > 0.0) {
                  //Serial.print(long_term_average);
                  //Serial.println(" lta");
                  

                  for (int j=0; j<ClapDataContainer::m_BUF_SIZE; j++) {
                    bufferMutex.lock();
                    cdc.buffer[j] = samplesB[internalBufferIndex%2][i-algo1_long_term_steps+j];
                    bufferMutex.unlock();
                  }

                  cdc.ts = millis() - 1000.0*algo1_short_term_s*algo1_long_term_mul;
                  cdc.trueOrFalse = false;
                  
                  double clap_likeness = short_term_average / long_term_average;
                  /*
                  if (clap_likeness > 0.0) {
                    Serial.print("short_term_average=");
                    Serial.println(short_term_average);
                  
                    Serial.print("long_term_average");
                    Serial.println(long_term_average);
                    Serial.print("ClapLikeness=");
                    Serial.println(clap_likeness);
                  }
                  */
                  if (clap_likeness > algo1_decision_threshold) {
                    if (algo1_during_clap == 0) {
                      algo1_during_clap = 1;
                      cdc.trueOrFalse = true;
                    }
                  }
                  else {
                    algo1_during_clap = 0;
                  }

      

                  ret.push_back(cdc);
                }
              }
            }

                  // Print the data for ML to the console.
            for (list<ClapDataContainer>::iterator it = ret.begin(); it != ret.end(); it++) {
              Serial.print("[");
              if (it->trueOrFalse) {
                Serial.print("true,");
              }
              else {
                Serial.print("false,");
              }
              for (int k=0; k<ClapDataContainer::m_BUF_SIZE; k++) {
                Serial.print(it->buffer[k]);
                Serial.print(",");
                yield();
              }
              Serial.print(",");
              Serial.println("]");        
            }
        }

        return NULL; 
    }

        void loop() {

      
        }
};

PrintTask printTask;
ReadingTask readingTask;

   void readingValues() {

  }

void setup() {
 /* 
 readingTask.create(0);
*/ 
 printTask.create(0);

 //bufferMutex.create();

  pinMode(27, INPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

 Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

      turnOnRedLed();
      yield();
      
      bufferMutex.lock();
      Serial.print("BufferRead_");
      Serial.println(bufferIndex%2);
      bufferMutex.unlock();
      
      
      for (int c=0; c<OVERLAP_NUM; c++) {
        bufferMutex.lock();
        // Copy the last OVERLAP_NUM samples to the new buffer
        samplesB[bufferIndex%2][c] = samplesB[(bufferIndex-1)%2][SAMPLES_NUM + c];
        bufferMutex.unlock();
        yield();
      }

      // Read SAMPLES_NUM new values and put them after the OVERLAP_NUM samples
      for (int c=0; c< SAMPLES_NUM; c++) {

        unsigned long startMicros= micros();
        float signalMax = 0.0;
        float signalMin = 1024.0;
      
      
        while(micros() - startMicros < LIMIT_MEASURE_TIME_US) {
          float v = analogRead(27);
          
          if(v <= 1024.0) {      // toss out spurious readings
            if(v > signalMax) {
              signalMax = v;  // save just the max levels
            } else if(v < signalMin) {
              signalMin = v;  // save just the min levels
            }
          }
          yield();
        }
      
        if(signalMax == 0 && signalMin == 1024) {
          signalMax = 0;
          signalMin = 0;
        }
        
        float peakToPeak = (float)signalMax - (float)signalMin;
        float sample = 3.3 * (float)peakToPeak / 1024.0;
      
        bufferMutex.lock();
        // Start after OVERLAP_NUM samples
        samplesB[bufferIndex%2][OVERLAP_NUM+c] = sample;
        bufferMutex.unlock();

        yield();
        
      }
      
      turnOffRedLed();
      
      bufferMutex.lock();
      bufferIndex++;
      bufferMutex.unlock();

  yield();
}
