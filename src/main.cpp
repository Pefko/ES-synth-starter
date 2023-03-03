#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval
  volatile uint32_t currentStepSize;
  volatile uint8_t keyArray[7];
  volatile uint8_t knob3;
  volatile uint8_t rotation = 0;
  const char * globalKeySymbol;
  SemaphoreHandle_t keyArrayMutex;

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  //Key frequencies
  const int key_C = 262;
  const int key_Csharp = 277;
  const int key_D = 294;
  const int key_Dsharp = 311;
  const int key_E = 330;
  const int key_F = 349;
  const int key_Fsharp = 367;
  const int key_G = 392;
  const int key_Gsharp = 415;
  const int key_A = 440;
  const int key_Asharp = 466;
  const int key_B = 494;

  //Set key step sizes
  const uint32_t stepSizes [] = {51149156, 54077543, 57396381, 60715220, 64424509, 68133799, 71647864, 76528508, 81018701, 85899346, 90975216, 96441538};
  char * keyOrder [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

uint8_t readCols(){
  // digitalWrite(REN_PIN, HIGH);
  // digitalWrite(RA0_PIN, LOW);
  // digitalWrite(RA1_PIN, LOW);
  // digitalWrite(RA2_PIN, LOW);

  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  // uint8_t col = digitalRead(C0_PIN) ? 0 : (digitalRead(C1_PIN) ? 1 : (digitalRead(C2_PIN) ? 2 : (digitalRead(C3_PIN) ? 3 : 100)));

  uint8_t result = 0;
  result |= (c0 << 0);
  result |= (c1 << 1);
  result |= (c2 << 2);
  result |= (c3 << 3);

  //uint8_t result = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0;
  return result;

}

uint8_t readKnobCols(){
  // digitalWrite(REN_PIN, HIGH);
  // digitalWrite(RA0_PIN, LOW);
  // digitalWrite(RA1_PIN, LOW);
  // digitalWrite(RA2_PIN, LOW);

  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);

  // uint8_t col = digitalRead(C0_PIN) ? 0 : (digitalRead(C1_PIN) ? 1 : (digitalRead(C2_PIN) ? 2 : (digitalRead(C3_PIN) ? 3 : 100)));

  uint8_t result1 = 0;
  uint8_t result2 = 0;
  result1 |= (c0 << 0);
  result1 |= (c1 << 1);
  // result2 |= (c2 << 2);
  // result2 |= (c3 << 3);

  // Serial.println(result1);

  //uint8_t result = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0;
  return result1;

}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  if(rotation==0){
    Vout=0;
  }
  else{
    Vout = Vout >> (8 - rotation);
  }
  analogWrite(OUTR_PIN, Vout + 128);
  // delayMicroseconds(2000);
  // analogWrite(OUTR_PIN, 0);
}

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    const uint8_t cols[] = {C0_PIN, C1_PIN, C2_PIN, C3_PIN};
    uint32_t lastStepSize = 0;
    uint8_t result1;
    uint8_t temprotate3 = rotation;
    bool turnRight;
    char * keysymbol = 0;
    // uint8_t key1, key2;
    uint8_t knob3before = knob3;
    const uint32_t * localStepSizes = stepSizes;
    
    //this is meant to wrap any keyArray accesses, 
    //maybe wrapping too much uncessary code rn (therefore locking for too long causing problems potentially)
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    for(uint8_t row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(10);
      if(row!=3){
        uint8_t keys = readCols();
        keyArray[row] = keys;
      }
      else if(row==3){
        result1 = readKnobCols();
        // Serial.println(result1);
        // Serial.println(knob3);
        // Serial.println("-------------------------");
      }
    }

    for(uint8_t i=0;i<3;i++){
      if(keyArray[i]==14){
        lastStepSize = localStepSizes[i*4];
        keysymbol = keyOrder[i*4];
      }
      else if(keyArray[i]==13){
        lastStepSize = localStepSizes[(i*4)+1];
        keysymbol = keyOrder[(i*4)+1];
      }
      else if(keyArray[i]==11){
        lastStepSize = localStepSizes[(i*4)+2];
        keysymbol = keyOrder[(i*4)+2];
      }
      else if(keyArray[i]==7){
        lastStepSize = localStepSizes[(i*4)+3];
        keysymbol = keyOrder[(i*4)+3];
      }
      else if(keyArray[i]!=15){
        lastStepSize = lastStepSize;
        keysymbol = keysymbol;
      }

      xSemaphoreGive(keyArrayMutex);
      
      // currentStepSize = lastStepSize;
      __atomic_store_n(&currentStepSize, lastStepSize, __ATOMIC_RELAXED);
      __atomic_store_n(&globalKeySymbol, keysymbol, __ATOMIC_RELAXED);
    }
    //add knob decoding here:
    if((knob3==0 && result1==1) || (knob3==3 && result1==2)){
      if(temprotate3 < 8){
        temprotate3 +=1;
      }
      turnRight = true;
    }
    else if((knob3==1 && result1==0) || (knob3==2 && result1==3)){
      if(temprotate3 > 0){
        temprotate3 -=1;
      }
      turnRight = false;
    }
    else if((knob3==0 && result1==3) || (knob3==1 && result1==2) || (knob3==2 && result1==1) || (knob3==3 && result1==0)){
      if(turnRight && temprotate3 < 8){
        temprotate3 +=1;
      }
      else if(!turnRight && temprotate3 > 0){
        temprotate3 -=1;
      }
    }
    __atomic_store_n(&rotation, temprotate3, __ATOMIC_RELAXED);
    knob3 = result1;
  }
}

void displayUpdateTask(void * pvParameters){

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.drawStr(100,30,"(._.)");
    
    u8g2.setCursor(2,20);
    for(uint8_t i=0;i<3;i++){
      u8g2.print(keyArray[i], HEX); 
    }
    
    // Store the result in the global variable
    u8g2.drawStr(2,30,globalKeySymbol);
    u8g2.setCursor(30,30);
    u8g2.print(rotation);
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void setup() {
  // put your setup code here, to run once:
  // uint32_t lastStepSize = globalLastStepSize;

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //timer that executes interrupt that calls sampleISR()
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		  /* Function that implements the task */
    "scanKeys",		    /* Text name for the task */
    64,      		      /* Stack size in words, not bytes */
    NULL,			        /* Parameter passed into the task */
    2,			          /* Task priority */
    &scanKeysHandle   /* Pointer to store the task handle */
  );	

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		  /* Function that implements the task */
    "displayUpdate",		    /* Text name for the task */
    64,      		            /* Stack size in words, not bytes */
    NULL,			              /* Parameter passed into the task */
    1,			                /* Task priority */
    &displayUpdateHandle    /* Pointer to store the task handle */
  );	


  keyArrayMutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}

void loop() {
}