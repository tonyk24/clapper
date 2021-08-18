#include <Arduino.h>
#include <Scheduler.h>
#include <Semaphore.h>

#include <list>

using namespace std;

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


#define LIMIT_MEASURE_TIME_US (1000000 / SAMPLE_RATE)

// Buffer to store the samples in
float samplesB[2][SAMPLES_NUM_OVERLAP];
// Keep track of which buffer we are in, 0 or 1
int bufferIndex = 1;

#define LED_RED_PIN 12
#define LED_BLUE_PIN 13



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

// Mutex for accessing the buffer and its index.
Semaphore bufferMutex(1);

bool readyAnalyze = false;

class ReadingTask : public Task {
  protected:
    void loop() {
      readingValues();
    }

    private:
    void readingValues() {
      turnOnRedLed();

      for (int c=0; c<OVERLAP_NUM; c++) {
        bufferMutex.wait();
        // Copy the last OVERLAP_NUM samples to the new buffer
        samplesB[bufferIndex%2][c] = samplesB[(bufferIndex-1)%2][SAMPLES_NUM + c];
        bufferMutex.signal();
        
      }

      // Read SAMPLES_NUM new values and put them after the OVERLAP_NUM samples
      for (int c=0; c< SAMPLES_NUM; c++) {

        unsigned long startMicros= micros();
        float signalMax = 0.0;
        float signalMin = 1024.0;
      
      
        while(micros() - startMicros < LIMIT_MEASURE_TIME_US) {
          float v = analogRead(A0);
          
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
      
        bufferMutex.wait();
        // Start after OVERLAP_NUM samples
        samplesB[bufferIndex%2][OVERLAP_NUM+c] = sample;
        bufferMutex.signal();

        yield();
      }
      turnOffRedLed();
      
      bufferMutex.wait();
      bufferIndex++;
      readyAnalyze = true;
      bufferMutex.signal();
  }
} reading_task;

class EvalTask : public Task {
  protected:
    void loop() {
      bool analyze = false;
      bufferMutex.wait();
      analyze = readyAnalyze;
      bufferMutex.signal();
      
      if (analyze) {
        Serial.println("**********************************");
        
        //printSamples(0);
        //printSamples(1);
        
        int internalBufferIndex = 0;
        bufferMutex.wait();
        internalBufferIndex = bufferIndex-1;
        bufferMutex.signal();
        
        bool clap = doubleClapDetection(internalBufferIndex%2) % 2 == 1;
        
        //printSamples();
        
        static int bluePinState = LOW;

        // A double clap occured
         if (clap) {
          if (bluePinState == LOW) {
            bluePinState = HIGH;
            turnOnBlueLed();
          }
          else {
            bluePinState = LOW;
            turnOffBlueLed();
          }
        }

        bufferMutex.wait();
        readyAnalyze = false;
        bufferMutex.signal();
      }
    }

    void printSamples(int bufIndx) {
      Serial.print("[");
      for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
        Serial.print(",");
        bufferMutex.wait();
        Serial.print(samplesB[(bufIndx)%2][i]);
        bufferMutex.signal();
      }
      
      Serial.println("]");
    }


  int clapDetection(int bufIndx) {
    return singleClapDetection(bufIndx).size();
  }

  list<unsigned long> singleClapDetection(int bufIndx) {
    return clapDetectionAlgo4(samplesB[bufIndx]);
  }

  int clap_detection_counter = 0;
  list<unsigned long> ulListClaps;
  int dcd_threshold_min_ms = 100;
  int dcd_threshold_max_ms = 2000;
  
  int doubleClapDetection(int bufIndx) {
    int ret = 0;

    unsigned long s = millis();
    list<unsigned long> cur = singleClapDetection(bufIndx);
    unsigned long e = millis();

    //Serial.print(e-s);
    //Serial.println(" delta");
    
    // Concatenate latest claps to list of claps
    ulListClaps.insert(ulListClaps.end(), cur.begin(), cur.end());

    list<unsigned long>::iterator it = ulListClaps.begin();

    unsigned long first = *it;
    unsigned long second = 0;
    it++;
    while (it != ulListClaps.end()) {
      second = *it;
      //Serial.print(second-first);
      //Serial.println(" delta clap");
     if (second-first > dcd_threshold_min_ms &&
          second-first < dcd_threshold_max_ms) {
            Serial.println("Double clap detected");
            list<unsigned long>::iterator itFirst = it;
            itFirst--;
            ulListClaps.erase(itFirst);
            ulListClaps.erase(it);
            it = ulListClaps.begin();
            first = *it;
            it++;
            ret++;
          }
          else {
            first = second;
            it++;            
          }
    }

    return ret;
  }

float algo1_short_term_s = 0.0025;
float algo1_long_term_mul = 15;
int algo1_short_term_steps = algo1_short_term_s * SAMPLE_RATE;
int algo1_long_term_steps = algo1_long_term_mul * algo1_short_term_steps;

int algo1_during_clap = 0;

const double algo1_decision_threshold = 12.0;


list<unsigned long> clapDetectionAlgo1(float *samples) {
  list<unsigned long> ret;

  double short_term_average_sum = 0.0;
  double long_term_average_sum = 0.0;

  for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
    yield();
    if (i >= algo1_long_term_steps) {
      // Calculate long term average
      long_term_average_sum = 0.0;
      for (int j=i-algo1_long_term_steps; j<i; j++) {
         long_term_average_sum += samples[j];
      }

      double long_term_average = long_term_average_sum / (double)algo1_long_term_steps;

      short_term_average_sum = 0.0;
      // Calculate short term average
      for (int j=i-algo1_short_term_steps; j<i; j++) {
        short_term_average_sum += samples[j];
      }

      double short_term_average = short_term_average_sum / (double)algo1_short_term_steps;

      if (long_term_average > 0.0) {
        double clap_likeness = short_term_average / long_term_average;

        if (clap_likeness > 0.0) {
          Serial.print("short_term_average=");
          Serial.println(short_term_average);
        
          Serial.print("long_term_average");
          Serial.println(long_term_average);
          Serial.print("ClapLikeness=");
          Serial.println(clap_likeness);
        }
        if (clap_likeness > algo1_decision_threshold) {
          if (algo1_during_clap == 0) {
            algo1_during_clap = 1;
            ret.push_back(millis() - 1000.0*algo1_short_term_s*algo1_long_term_mul);
          }
        }
        else {
          algo1_during_clap = 0;
        }
      }
    }
  }

  return ret;
}

float algo2_short_term_s = 0.0025;
float algo2_long_term_mul = 15;

int algo2_short_term_steps = algo2_short_term_s * SAMPLE_RATE;
float algo2_threshold = 0.35;
int algo2_max_allowed_clap_duration = algo2_long_term_mul;
float algo2_decision_threshold = 1.0;

list<unsigned long> clapDetectionAlgo2(float *samples) {
  list<unsigned long> ret;
  bool clap = false;

  float short_term_average_sum = 0.0;
  float max_val = 0.0;
  int clap_duration = 0;
  
  //Serial.print("[");

  for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
    yield();
    if (i >= algo2_short_term_steps) {
      short_term_average_sum = 0.0;
      
      // Calculate short term average
      for (int j=i-algo2_short_term_steps; j<i; j++) {
        short_term_average_sum += samples[j];
      }

      float short_term_average = short_term_average_sum /  (float)algo2_short_term_steps;

      if (short_term_average > 0.0) {
        Serial.print(short_term_average);
        Serial.println(" sta");
      }
      
      if (short_term_average > algo2_threshold) {
        max_val = max(max_val, short_term_average-algo2_threshold);
        clap_duration++;
      }

      if (clap_duration > algo2_max_allowed_clap_duration) {
        clap_duration = 0;
        max_val = 0;
        clap = false;
        //Serial.println("clap false");
        break;
      }

      if (clap_duration > 0) {
        float clap_likeness = (10.0 * max_val) * (10.0 * max_val) / (float)clap_duration;
        if (clap_likeness > 0.0) {
            Serial.print(clap_likeness, 4);
            Serial.println(" cl");
        }
        if (clap_likeness > algo2_decision_threshold) {
          clap = true;
        }
      }
      
    }
  }

  if (clap) {
    ret.push_back(millis() - 1000.0*algo2_short_term_s*algo2_long_term_mul);
  }
  
  return ret;
}

float algo3_short_term_s = 0.0025;
float algo3_long_term_mul = 7.0;

int algo3_short_term_steps = algo3_short_term_s * SAMPLE_RATE;
int algo3_long_term_steps = algo3_long_term_mul * algo3_short_term_steps;

float algo3_threshold_constant = 0.22;
float algo3_decision_threshold = 0.4;
int max_allowed_clap_duration = 6;

int algo3_during_clap = 0;

bool log = true;

list<unsigned long> clapDetectionAlgo3(float *samples) {
  list<unsigned long> ret;

  double short_term_average_sum = 0.0;
  double long_term_average_sum = 0.0;

  int last_clap_duration = 0;
  int clap_duration = 0;

  //Serial.print("[");
  float max_val = 0;
  for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
    //Serial.print(",");
    //Serial.print(samples[i]);
    yield();
    if (i >= algo3_long_term_steps) {
      // Calculate long term average
      long_term_average_sum = 0.0;
      for (int j=i-algo3_long_term_steps; j<i; j++) {
         long_term_average_sum += samples[j];
      }

      float long_term_average = long_term_average_sum / (float)algo3_long_term_steps;

      short_term_average_sum = 0.0;
      // Calculate short term average
      for (int j=i-algo3_short_term_steps; j<i; j++) {
        short_term_average_sum += samples[j];
      }

      float short_term_average = short_term_average_sum / (float)algo3_short_term_steps;
      float threshold = algo3_threshold_constant + long_term_average;


      last_clap_duration = clap_duration;

      if (short_term_average > 0.0 && log) {
        Serial.print(short_term_average);
        Serial.println(" sta");
        Serial.print(long_term_average);
        Serial.println(" lta");
      }
    
      if (short_term_average > threshold) {
        max_val = max(max_val, short_term_average-threshold);

        clap_duration++;  
      }
      else
        clap_duration = 0;
      
      if (clap_duration == 0 && clap_duration != last_clap_duration)
      {
        float clap_likeness = (10.0*max_val) * (10.0*max_val) / last_clap_duration;

        if (clap_likeness > 0.0 && log) {
            Serial.print(max_val, 4);
            Serial.println(" max_val");
            Serial.print(clap_likeness, 4);
            Serial.println(" cl");
        }

        if (clap_likeness > algo3_decision_threshold)
        {
          ret.push_back(millis() - 1000.0*algo3_short_term_s*algo3_long_term_mul);
          max_val = 0;
        }
      }
    }
  }
  

  return ret;
}

float algo4_short_term_s = 0.0025;
float algo4_long_term_mul = 14.0;

int algo4_short_term_steps = algo4_short_term_s * SAMPLE_RATE;
int algo4_long_term_steps = algo4_long_term_mul * algo4_short_term_steps;

float algo4_threshold_constant = 0.22;
float algo4_decision_threshold = 1.0;
int algo4_max_allowed_clap_duration = 12;

bool algo4_log = true;

list<unsigned long> clapDetectionAlgo4(float *samples) {
  list<unsigned long> ret;

  double short_term_average_sum = 0.0;
  double long_term_average_sum = 0.0;

  int clap_duration = 0;
  int last_clap_duration = 0;
  float max_val = 0;
  Serial.print("[");
  for (int i=0; i<SAMPLES_NUM_OVERLAP; i++) {
    yield();
    if (i >= algo4_long_term_steps) {
      // Calculate long term average
      long_term_average_sum = 0.0;
      for (int j=i-algo4_long_term_steps; j<i; j++) {
         long_term_average_sum += samples[j];
      }

      float long_term_average = long_term_average_sum / (double)algo3_long_term_steps;

      short_term_average_sum = 0.0;
      // Calculate short term average
      for (int j=i-algo4_short_term_steps; j<i; j++) {
        short_term_average_sum += samples[j];
      }

      float short_term_average = short_term_average_sum / (double)algo4_short_term_steps;
      float threshold = algo4_threshold_constant + long_term_average;
      last_clap_duration = clap_duration;
      

      if (short_term_average > 0.0 && algo4_log) {
        Serial.print(threshold);
        Serial.println(" th");
        Serial.print(long_term_average);
        Serial.println(" lta");
        Serial.print(short_term_average);
        Serial.println(" sta");
      }

      if (short_term_average > threshold) {
        max_val = max(max_val, short_term_average-threshold);

        clap_duration++;
      }
      else
      {
        clap_duration = 0;
      }

      if (clap_duration == 0 
          && clap_duration != last_clap_duration 
          && last_clap_duration <= algo4_max_allowed_clap_duration) {
            //Serial.println("Detection...");
        float clap_likeness = 10.0 * max_val * 10.0 * max_val / (float)last_clap_duration;

        if (clap_likeness > 0.0 && algo4_log) 
        {
            Serial.print(max_val, 4);
            Serial.println(" cl");
            Serial.print(clap_likeness, 4);
            Serial.println(" cl");
        }

        if (clap_likeness > algo4_decision_threshold)
        {
            // Save the timestamp in milliseconds for when long_term_average was started to be measured.
            ret.push_back(millis() - 1000.0*algo4_short_term_s*algo4_long_term_mul);
        }
      }
    }
  }
  

  return ret;
}
} eval_task;


void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  Serial.begin(115200);

  bufferMutex.wait();
  for (int j=0; j<SAMPLES_NUM_OVERLAP; j++) {
    samplesB[0][j] = 0.0;
    samplesB[1][j] = 0.0;
    yield();
  }
  bufferMutex.signal();
  

  
  delay(1000);

  Scheduler.start(&reading_task);
  Scheduler.start(&eval_task);

  Scheduler.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
