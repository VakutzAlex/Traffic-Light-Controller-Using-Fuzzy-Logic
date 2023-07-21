#include "FuzzyGbr.h"

//Pins for HC-SR04
#define trigPin1 11
#define trigPin2 10
#define echoPin1 13
#define echoPin2 12

//LEDs
#define GreenLED 4
#define RedLED 8
#define YellowLED 7

//Variables
float duration;
float queueIN; // current value for Queue
float arrivalIN; // current value for Arrival

//Function prototype
float readDist(int trigPin, int echoPin);
float mn(float a, float b);

FuzzyGbr MDComp;

const int NUM_MF_Queue = 4;  // number of fuzzy sets for Queue, input 
const int NUM_MF_Arrival = 4; // number of fuzzy sets Arrival, input
const int NUM_MF_ExtGreen = 4;    // number of fuzzy sets for TimeExt, output 
const int MIN_ExtGreen = 0; // lower bound for Speed
const int MAX_ExtGreen = 120;  // uper bound for Speed
// ---------------------------
const int NUM_SAMPLES = 1000;//number of samples to discretize Speed
float stepExtGreen; // necessary to descretize the Speed variable
float sExtGreen; //sample of the Speed; 
// -------------------------------
float accMD, accMdSample, accMdOut, ExtGreenStar; // Necessary for inference, aggregation and defuzzification
float mdOut, mdOutStarMax, mdOutStar[4]; // Necessary for inference, aggregation and defuzzification

// -------------   Define the Knowledge Base of the fuzzy system -----------------
// ......... Define the Data base - fuzzy sets ............
// define mf for Visibility
String  queueMfType[NUM_MF_Queue] = {"trimf","trimf","trimf", "trapmf"}; // mf type
float queueMF[NUM_MF_Queue][4]= {         {0, 0, 5},   // parameters for the 1st mf: 
                                          {0, 5, 10},   // 2nd mf                    
                                          {5, 10, 15}, // 3rd mf              
                                          {10, 15, 20, 20} };   // 4th mf          
// define mf for RoadQuality
String  arrivalMfType[NUM_MF_Arrival] = {"trimf","trimf","trimf", "trapmf"}; // mf type
float arrivalMF[NUM_MF_Arrival][4]= {     {0, 0, 6},   // parameters for the 1st mf: 
                                          {0, 6, 12},   // 2nd mf                    
                                          {6, 12, 18}, // 3rd mf              
                                          {12, 16, 24, 24} };   // 4th mf  

String  ExtGreenMfType[NUM_MF_ExtGreen] = {"singlemf","singlemf","singlemf", "singlemf"}; // mf type
float ExtGreenMF[NUM_MF_ExtGreen][4]= {    {0},   // parameters for the 1st mf: 
                                           {40},   // 2nd mf                    
                                           {60},    // 3rd mf              
                                           {120} };   // 4th mf
// ......... End of Data base - fuzzy sets ............

// ****** Define the Rule base ********
/* In fact define only an ordered array (rule1, rule2,...,)containing the index 
  of the output fuzzy set for each rule
  For example: 0 for the first output mf; 4 for the fifth output mf.
  SLF Rule base:
  1.  If Queue is VFew     and Arrival is AlmostNone        then Speed is Zero        0 
  2.  If Queue is VFew     and Arrival is Few               then Speed is SmallExt    40   
  3.  If Queue is VFew     and Arrival is Many              then Speed is MedExt      60
  4.  If Queue is VFew     and Arrival is VeryMany          then Speed is LargeExt    120   
  5.  If Queue is Few      and Arrival is AlmostNone        then Speed is Zero        0
  6.  If Queue is Few      and Arrival is Few               then Speed is SmallExt    40   
  7.  If Queue is Few      and Arrival is Many              then Speed is MedExt      60
  8.  If Queue is Few      and Arrival is VeryMany          then Speed is MedExt      60 
  9.  If Queue is Med      and Arrival is AlmostNone        then Speed is Zero        0
  10. If Queue is Med      and Arrival is Few               then Speed is Zero        0
  11. If Queue is Med      and Arrival is Many              then Speed is SmallExt    40
  12. If Queue is Med      and Arrival is VeryMany          then Speed is MedExt      60 
  13. If Queue is Many     and Arrival is AlmostNone        then Speed is Zero        0
  14. If Queue is Many     and Arrival is Few               then Speed is Zero        0
  15. If Queue is Many     and Arrival is Many              then Speed is SmallExt    40
  16. If Queue is Many     and Arrival is VeryMany          then Speed is MedExt      60   
 */ 
int rules[] = {0, 40, 60, 120, 0, 40, 60, 60, 0, 0, 40, 60, 0, 0, 40, 60};  
// ****** End of Rule Base *******

// -------------   End of the Knowledge Base of the fuzzy system -----------------

float mdQueue[NUM_MF_Queue]; // membership degree of the visibilityIn to each of its fuzzy sets
float mdArrival[NUM_MF_Arrival]; // membership degree of the qualityRoadIn to each of its fuzzy sets
float fdRule [NUM_MF_Queue*NUM_MF_Arrival]; // array for firing degree of the rules 
float fdActiveRule[4]; // only firing degree of active rules (fd  > 0), maximum 4 active rules 
int   idxActiveMf[4]; // index of the mf corsponding to active rules only; corellated with fdActiveRule

int i,j,k,m;  // some indexes

float computeMD (float inpVal, float params[], String mfType){
float md;
if (mfType == "trimf")  {    //call triMf function in the FuzzyGbr library
    md = MDComp.triMf(inpVal, params); 
  } 
  if (mfType == "trapmf")  {  //call trapMf function in FuzzyGbr library
    md = MDComp.trapMf(inpVal, params); 
  }
  if (mfType == "singlemf")  { //call singleMf function in the FuzzyGbr library
    md = MDComp.singleMf(inpVal, params); 
  }  
  return md;
}

void setup() {
  Serial.begin(9600);
  
  //triggers
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);

  //echoes
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);

  //LEDs
  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
}

void loop() {
  float firstSens, secondSens;
  float passedCar;
  delay(5);
  firstSens=readDist(trigPin1,echoPin1);
  delay(5);
  secondSens=readDist(trigPin2,echoPin2);

  Serial.println(firstSens);

  if((queueIN+arrivalIN)-passedCar!=0){
    queueIN=(queueIN+arrivalIN)-passedCar;
    passedCar=0;
  }
  else{
    queueIN=0;
    passedCar=0;
  }
  
  arrivalIN=0;
  i=0;
  
  Serial.println("Traffic light is red");
  digitalWrite(RedLED, HIGH);
  //traffic light is red -- compute queueIN
  while(i < 30){
    if(readDist(trigPin1,echoPin1)<6){
     if(queueIN>=20){
        queueIN=20;
        Serial.println("Queue full");
      }
    else{
     queueIN++; 
    }
    }
    i++;
    delay(200);
  }
  digitalWrite(RedLED, LOW);
  delay(200);

  digitalWrite(YellowLED, HIGH);
  delay(200);
  digitalWrite(YellowLED, LOW);
  delay(200);

  i=0;
  Serial.println("Traffic light is green");
  digitalWrite(GreenLED, HIGH);
  delay(500);
  //traffic light is green -- compute arrivalIN
  while(i < 20){
    if(readDist(trigPin1,echoPin1)<6){
     if(arrivalIN>=24){
        arrivalIN=24;
      }
    else{
     arrivalIN++; 
    }
    delay(200);
    }
    if(readDist(trigPin2,echoPin2) < 6){
      passedCar++;
    }
    delay(200);
    i++;
  }
  //digitalWrite(GreenLED, LOW);

//queueIN=0;
//arrivalIN=24;

Serial.print("Number of cars in the queue at the traffic light, while red: "); Serial.println(queueIN);
Serial.print("Number of cars arriving at the traffic light, while green: "); Serial.println(arrivalIN);
Serial.print("Number of cars that passed traffic light, while green: "); Serial.println(passedCar);

/* Compute membership degrees for curent inputs for all its fuzzy sets */
// Queue input
Serial.println("Membership degrees for Queue input");
for (i = 0; i < NUM_MF_Queue; i++)  {
  mdQueue[i] = computeMD(queueIN, queueMF[i], queueMfType[i]); // call the function to compute md
Serial.print(mdQueue[i]);  Serial.print("    ");
}
Serial.println("    "); Serial.println("    ");

// Arrival input
Serial.println("Membership degrees for Arrival input");
for (i = 0; i < NUM_MF_Arrival; i++)  {
  mdArrival[i] = computeMD(arrivalIN, arrivalMF[i], arrivalMfType[i]); // call the function to compute md
Serial.print(mdArrival[i]);  Serial.print("    ");
}
Serial.println("    "); Serial.println("    ");

// compute the firing degree of the rules
k = 0; m = 0;
Serial.println("Firing degree of the rules");
for (i = 0; i < NUM_MF_Queue; i++) {
  for (j = 0; j < NUM_MF_Arrival; j++) {
    fdRule[k] = mn(mdQueue[i], mdArrival[j]);               // prod for AND operator for multiple antecedents
    Serial.print(fdRule[k]);  Serial.print(" ");
    if (fdRule[k]){
      fdActiveRule[m] = fdRule[k];
      idxActiveMf[m] = rules[k];
      m++;
    }
    k++;
  }
}

Serial.println("    "); Serial.println("    ");
Serial.println("Firing degree of active rules only:");
for (i = 0; i<4; i++){
Serial.print(fdActiveRule[i]); Serial.print("  ");
}
Serial.println("    "); Serial.println("    ");
Serial.println("Index of output mf for active rules only:");
for (i = 0; i<4; i++){
Serial.print(idxActiveMf[i]); Serial.print("  ");
}
// ~~~~~~~~~~~~~~~~ Start of Inference, Agreggation, Defuzzification ~~~~~~~~~~~~~  
 stepExtGreen = (MAX_ExtGreen - MIN_ExtGreen)/(float) NUM_SAMPLES;//discretise the extension
 accMD = 0;
 float sum=0;  
 accMdSample = 0;
 for (sExtGreen = MIN_ExtGreen; sExtGreen <= MAX_ExtGreen; sExtGreen = sExtGreen + stepExtGreen){
   for (i = 0; i < 4; i++) {
   mdOut=computeMD(sExtGreen,ExtGreenMF[idxActiveMf[i]],ExtGreenMfType[idxActiveMf[i]]);
   mdOutStar[i] = mn(fdActiveRule[i], mdOut); 
    }
  mdOutStarMax = 0;
  for (i = 0; i < 4; i++){
    ExtGreenStar=ExtGreenStar+fdActiveRule[i]*idxActiveMf[i];
    sum+=fdActiveRule[i];
  }
  accMD = accMD + mdOutStarMax;
  accMdSample = accMdSample + mdOutStarMax * sExtGreen;
 }
 ExtGreenStar = ExtGreenStar / sum;
 Serial.println("  "); Serial.println("  ");
 Serial.print("The current ExtensionTime* is:    "); Serial.println(ExtGreenStar);

 while((int)ExtGreenStar!=0){
  ExtGreenStar--;
  delay(200);
 }
 digitalWrite(GreenLED, LOW);
 
// ~~~~~~~~~~~~~~~~ End of Inference, Agreggation, Defuzzification ~~~~~~~~~~~~~~~~~~~~

Serial.println("    ");
Serial.println("===========================================================================");
Serial.println("    "); Serial.println("    ");
exit(0); // exit the loop
}

float readDist(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);  //makes sure triggerPin is low
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //send out a pulse to HC1
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  //turnoff trigger pin

  duration = pulseIn(echoPin, HIGH);

  return duration/58.2;
}

float mn(float a, float b)
{
    return (a>b) ? b:a;
}
