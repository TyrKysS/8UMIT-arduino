#include <math.h>
#include <DS3231.h>
#include <BH1750.h>
#include <Wire.h>
#include <SPI.h>

#define pinLed 6
#define BUTTON_PIN 2

DS3231 rtc;
RTCDateTime DateTime;
BH1750 lightMeter;

/******************************************************************
 * Network Configuration - customized per network 
 ******************************************************************/
const int PatternCount = 14;
const int InputNodes = 3;
const int HiddenNodes = 4;
const int OutputNodes = 2;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;
bool trainedData = false;

/******************************************************************
 * Training data
 ******************************************************************/
const float Input[PatternCount][InputNodes] = { // Input matrix data
  // BtnState, Clock, Light
  {     0,       1,     0   },
  {     0,      0.70,   0   },
  {     0,      0.70,   1   },
  {     0,      0.50,   1   },
  {     0,      0.17,  0.91 },
  {     0,      0.08,   0   },
  {     0,      0.23,  0.25 },
  {     1,        1,    0   },
  {     1,      0.70,   1   },
  {     1,      0.70,   0   },
  {     1,      0.23,  0.25 },
  {     1,      0.17,  0.91 },
  {     1,      0.09,   1   },
  {     1,        0,    1   }
};

const float Target[PatternCount][OutputNodes] = { //Output matrix data
  // LightIntens, BtnState
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     0,          0     },
  {     1,          1     },
  {     0,          0     },
  {     0.25,       1     },
  {     1,          1     },
  {     0.2,        1     },
  {     0,          0     }
};

/******************************************************************
 * End Network Configuration
 ******************************************************************/

int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long TrainingCycle;
float Rando;
float Error;
float Accum;


float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

/******************************************************************
 * Variables
 ******************************************************************/
int inputData = 0, previousInputData = 0, der = 0;
int clapCounter = 0;
int btnStatus = 0;
int index = 0;
const int patternLength = 10;

/******************************************************************
 * Acustic configuration
 ******************************************************************/
int minusThreshold = -300, plusThreshold = 300, tolerance = 200;
int time = 0, previousTime = 0, deadlineTime = 0;
bool clap = false, preclap = false, passTreshold = false;

/******************************************************************
 * Acustic patterns
 ******************************************************************/
int rows = 6, colls = patternLength;
int inputDataArray[patternLength];
int patterns[6][patternLength] = {
  { -326, 221, 262, -159, -15, 11, -83, -27, 26, -17 },
  { 440, -235, 240, -485, 483, -154, -82, 197, -187, -37 },
  { 346, -22, -207, 139, -21, -5, -134, 37, 13, 189 },
  { -289, 224, 163, -95, -220, 200, 208, -263, 70, 0 },
  { -413, 280, 85, -307, -28, 57, 68, 56, 97, -12 },
  { 484, -159, -126, 185, -387, 371, -137, -56, 11, -82 }
};

void setup() {
  Serial.begin(9600);
  rtc.begin();
  Wire.begin();
  lightMeter.begin();


  randomSeed(analogRead(3));
  ReportEvery1000 = 1;
  for (p = 0; p < PatternCount; p++) {
    RandomizedIndex[p] = p;
  }

  
/******************************************************************
 * Set actual time
 ******************************************************************/
  rtc.setDateTime(__DATE__, __TIME__);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

/******************************************************************
 * Print all patterns
 ******************************************************************/
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < colls; j++) {
      Serial.print(patterns[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();

  inputData = analogRead(A0);
}


void loop() {
  if (trainedData) {
    clapDectect();
    if (clapCounter == 2 && btnStatus == 0) {
      btnStatus = 1;
      clapCounter = 0;
    } else if (clapCounter == 2 && btnStatus == 1) {
      btnStatus = 0;
      clapCounter = 0;
    }

/******************************************************************
 * Btn settings
 ******************************************************************/
    if (digitalRead(BUTTON_PIN) == 0) { // If btn is pressed, rise variable
      btnStatus++;
      delay(500);
    }
    if (btnStatus == 2){  // if btnVariable equals 2, reset it
      btnStatus = 0;
    }
    
/******************************************************************
 * Reading data from sensors
 ******************************************************************/
    DateTime = rtc.getDateTime();
    double lux = lightMeter.readLightLevel();
    
/******************************************************************
 * Converting input data into range from 0 to 1
 ******************************************************************/
    double actualTime = hours2min(DateTime.hour) + DateTime.minute;
    float convertedActualTime = PwmToPercent(actualTime, 0, 1439) / 100;
    float convertedInputLight = PwmToPercent(lux, 0, 50) / 100;
    
/******************************************************************
 * Transforming input into output
 ******************************************************************/
    inputToOutput(btnStatus, convertedActualTime, convertedInputLight);
    double finalOutput = percentToPWM(Output[0] * 100);
    analogWrite(pinLed, finalOutput);
    btnStatus = round(Output[1]);
    delay(5);
  } else {
    trainData();
  }
}

void trainData() {
  /******************************************************************
* Initialize HiddenWeights and ChangeHiddenWeights 
******************************************************************/
  for (i = 0; i < HiddenNodes; i++) {
    for (j = 0; j <= InputNodes; j++) {
      ChangeHiddenWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }

  /******************************************************************
* Initialize OutputWeights and ChangeOutputWeights
******************************************************************/
  for (i = 0; i < OutputNodes; i++) {
    for (j = 0; j <= HiddenNodes; j++) {
      ChangeOutputWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }
  Serial.println("Initial/Untrained Outputs: ");
  toTerminal();

  /******************************************************************
* Begin training 
******************************************************************/
  for (TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++) {
    /******************************************************************
* Randomize order of training patterns
******************************************************************/
    for (p = 0; p < PatternCount; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p];
      RandomizedIndex[p] = RandomizedIndex[q];
      RandomizedIndex[q] = r;
    }
    Error = 0.0;
    /******************************************************************
* Cycle through each training pattern in the randomized order
******************************************************************/
    for (q = 0; q < PatternCount; q++) {
      p = RandomizedIndex[q];

      /******************************************************************
* Compute hidden layer activations
******************************************************************/
      for (i = 0; i < HiddenNodes; i++) {
        Accum = HiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++) {
          Accum += Input[p][j] * HiddenWeights[j][i];
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum));
      }

      /******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/
      for (i = 0; i < OutputNodes; i++) {
        Accum = OutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++) {
          Accum += Hidden[j] * OutputWeights[j][i];
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum));
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]);
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
      }

      /******************************************************************
* Backpropagate errors to hidden layer
******************************************************************/
      for (i = 0; i < HiddenNodes; i++) {
        Accum = 0.0;
        for (j = 0; j < OutputNodes; j++) {
          Accum += OutputWeights[i][j] * OutputDelta[j];
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
      }


      /******************************************************************
* Update Inner-->Hidden Weights
******************************************************************/
      for (i = 0; i < HiddenNodes; i++) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
        }
      }

      /******************************************************************
* Update Hidden-->Output Weights
******************************************************************/
      for (i = 0; i < OutputNodes; i++) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
          OutputWeights[j][i] += ChangeOutputWeights[j][i];
        }
      }
    }

    /******************************************************************
* Every 1000 cycles send data to terminal for display
******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0) {
      Serial.println();
      Serial.println();
      Serial.print("TrainingCycle: ");
      Serial.print(TrainingCycle);
      Serial.print("  Error = ");
      Serial.println(Error, 5);

      toTerminal();

      if (TrainingCycle == 1) {
        ReportEvery1000 = 999;
      } else {
        ReportEvery1000 = 1000;
      }
    }


    /******************************************************************
* If error rate is less than pre-determined threshold then end
******************************************************************/
    if (Error < Success) break;
  }
  Serial.println();
  Serial.println();
  Serial.print("TrainingCycle: ");
  Serial.print(TrainingCycle);
  Serial.print("  Error = ");
  Serial.println(Error, 5);

  toTerminal();

  Serial.println();
  Serial.println();
  Serial.println("Training Set Solved! ");
  Serial.println("--------");
  Serial.println();
  Serial.println();
  analogWrite(pinLed, 255);
  delay(100);
  analogWrite(pinLed, 0);
  delay(50);
  analogWrite(pinLed, 255);
  delay(100);
  analogWrite(pinLed, 0);
  trainedData = true;
}
void toTerminal() {
  for (p = 0; p < PatternCount; p++) {
    Serial.println();
    Serial.print("  Training Pattern: ");
    Serial.println(p);
    Serial.print("  Input ");
    for (i = 0; i < InputNodes; i++) {
      Serial.print(Input[p][i], DEC);
      Serial.print(" ");
    }
    Serial.print("  Target ");
    for (i = 0; i < OutputNodes; i++) {
      Serial.print(Target[p][i], DEC);
      Serial.print(" ");
    }

    /******************************************************************
* Compute hidden layer activations
******************************************************************/
    for (i = 0; i < HiddenNodes; i++) {
      Accum = HiddenWeights[InputNodes][i];
      for (j = 0; j < InputNodes; j++) {
        Accum += Input[p][j] * HiddenWeights[j][i];
      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum));
    }

    /******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/
    for (i = 0; i < OutputNodes; i++) {
      Accum = OutputWeights[HiddenNodes][i];
      for (j = 0; j < HiddenNodes; j++) {
        Accum += Hidden[j] * OutputWeights[j][i];
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum));
    }
    Serial.print("  Output ");
    for (i = 0; i < OutputNodes; i++) {
      Serial.print(Output[i], 5);
      Serial.print(" ");
    }
  }
}
void inputToOutput(double In1, double In2, double In3) {
  float TestInput[] = { 0, 0, 0 };
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/
  for (i = 0; i < HiddenNodes; i++) {
    Accum = HiddenWeights[InputNodes][i];
    for (j = 0; j < InputNodes; j++) {
      Accum += TestInput[j] * HiddenWeights[j][i];
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum));
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/
  for (i = 0; i < OutputNodes; i++) {
    Accum = OutputWeights[HiddenNodes][i];
    for (j = 0; j < HiddenNodes; j++) {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum));
  }
}

/******************************************************************
  * Clap decentor
  ******************************************************************/
void clapDectect() {
  previousInputData = inputData;
  previousTime = time;
  time++;

  inputData = analogRead(A0); // reading data from microphone
  der = derivation(previousTime, previousInputData, time, inputData); // derivating input data

  // if derivated speed is higher then treshold
  if ((der > plusThreshold || der < minusThreshold) && passTreshold == false) {
    deadlineTime = time;
    passTreshold = true;
  }

  if (passTreshold) {
    inputDataArray[index] = der; // Save new sample into array
    index++;
  }
  if (index == patternLength) {
    passTreshold = false;
    for (int i = 0; i < patternLength; i++) {
      Serial.print(inputDataArray[i]); //print new sample
      Serial.print(" ");
    }
    Serial.println();
    int lockRow = 0;
    // find the similarity of the first values between the sample and the patterns
    for (int i = 0; i < rows; i++) {
      if ((abs(patterns[i][0]) - abs(inputDataArray[0])) < tolerance) { 
        preclap = true;
        lockRow = i;
      }
    }
    // Find similarity between sample and pattern
    if (preclap) {
      for (int i; i < colls; i++) {
        if ((abs(patterns[lockRow][i]) - abs(inputDataArray[i])) < tolerance)
          clap = true;
        else
          clap = false;
      }
    }
    preclap = false;
    index = 0;

    if (clap) { // if clap detected, rise clap variable
      clapCounter++;
    }
  }
  // since detected a clap, reset all variables in 100 ms
  if (time - deadlineTime >= 100) {
    deadlineTime = 0;
    clapCounter = 0;
  }
}


int percentToPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  int pwm = (percent * 255) / 100;
  return pwm;
}
float PwmToPercent(float actualValue, float min, float max) {
  float ret = 0.0f;
  ret = 100 - (((actualValue - min) / (max - min)) * 100);
  if (ret <= 0)
    return 0;
  return ret;
}
double hours2min(int hour) {
  return hour * 60;
}
int derivation(int previousTime, int previousInputData, int time, int inputData) {
  return (inputData - previousInputData) / (time - previousTime);
}
