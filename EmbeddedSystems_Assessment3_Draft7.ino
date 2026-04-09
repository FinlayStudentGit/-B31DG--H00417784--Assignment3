/*****************************************************************************
----------------------------===<Header Files>===------------------------------
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>



/*****************************************************************************
------------------------===<Defines for monitor>===---------------------------
*****************************************************************************/
#define BOARD_PROFILE_240MHZ 1    //establishing frequency differances between each ESP32 board
#define BOARD_PROFILE_160MHZ 2

#ifndef BOARD_PROFILE 
#define BOARD_PROFILE BOARD_PROFILE_240MHZ  //Use 240MHZ CPU
#endif

#if BOARD_PROFILE == BOARD_PROFILE_240MHZ   //use these frequencies in the code, if 240MHZ CPU is used
static const uint32_t BUDGET_A_CYCLES = 672000u;  
static const uint32_t BUDGET_B_CYCLES = 960000u;
static const uint32_t BUDGET_AGG_CYCLES = 480000u;
static const uint32_t BUDGET_C_CYCLES = 1680000u;
static const uint32_t BUDGET_D_CYCLES = 960000u;
static const uint32_t BUDGET_S_CYCLES = 600000u;
#endif

static const uint32_t FINAL_REPORT_AFTER_SECONDS = 1u;  //Setting report period



/*****************************************************************************
-----------------------------===<Data setup>===-------------------------------
*****************************************************************************/
const uint8_t app_cpu = 1;  //Selecting to use CPU 1

const uint8_t SYNC = 19;  //setting pin aliases 
const uint8_t IN_A = 18; 
const uint8_t IN_B = 5; 
const uint8_t IN_S = 17; 
const uint8_t IN_MODE = 16;  
const uint8_t ACK_A = 4; 
const uint8_t ACK_B = 27;
const uint8_t ACK_AGG = 26;
const uint8_t ACK_C = 25;
const uint8_t ACK_D = 33;
const uint8_t ACK_S = 32;

volatile uint16_t countA = 0;  //A and B pulse count variables
volatile uint16_t countB = 0;

volatile uint32_t tokenA = 0;  //A and B tokens defined globelly for AGG task access
volatile uint32_t tokenB = 0;

uint16_t IDA = 0;  //task indexes
uint16_t IDB = 0;
uint16_t IDAGG = 0;
uint16_t IDC = 0;
uint16_t IDD = 0;
uint16_t IDS = 0;

volatile bool modeFlag = true;    //This will later be check during tasks C and D to toggle task C and D ON/OFF

SemaphoreHandle_t syncSemaphore;  //
SemaphoreHandle_t sSemaphore;
SemaphoreHandle_t xTokenMutex;



/*****************************************************************************
------------------------------===<Monitor>===---------------------------------
*****************************************************************************/
struct TaskStats {
  uint32_t jobs = 0;
  uint32_t misses = 0;
  uint64_t max_exec_us = 0;
  int64_t worst_lateness_us = 0;
  uint64_t start_us = 0;
  uint64_t release_us = 0;
  uint64_t deadline_us = 0;
  bool active = false;
};

class TimingMonitor {
public:
  void synch() {
    t0_ = micros();
    resetTask(a_);
    resetTask(b_);
    resetTask(agg_);
    resetTask(c_);
    resetTask(d_);
    resetTask(s_);
  }

  void notifySRelease() {}

  void setPeriodicReportEverySeconds(uint32_t s) {
    periodic_us_ = (uint64_t)s * 1000000ULL;
    next_report_ = t0_ + periodic_us_;
  }

  void setFinalReportAfterSeconds(uint32_t s) {
    final_report_ = t0_ + ((uint64_t)s * 1000000ULL);
  }

  bool pollReports() {
    uint64_t now = micros();
    if (periodic_us_ && now >= next_report_) {
      report();
      next_report_ += periodic_us_;
    }
    if (!final_printed_ && now >= final_report_) {
      printFinalReport();
      final_printed_ = true;
      return true;
    }
    return false;
  }

  bool allDeadlinesMet() {
    return a_.misses == 0 && b_.misses == 0 && agg_.misses == 0 &&
           c_.misses == 0 && d_.misses == 0 && s_.misses == 0;
  }

  void beginTaskA(uint32_t id){ beginTask(a_, id, 10000); }
  void endTaskA(){ endTask(a_); }

  void beginTaskB(uint32_t id){ beginTask(b_, id, 20000); }
  void endTaskB(){ endTask(b_); }

  void beginTaskAGG(uint32_t id){ beginTask(agg_, id, 20000); }
  void endTaskAGG(){ endTask(agg_); }

  void beginTaskC(uint32_t id){ beginTask(c_, id, 50000); }
  void endTaskC(){ endTask(c_); }

  void beginTaskD(uint32_t id){ beginTask(d_, id, 50000); }
  void endTaskD(){ endTask(d_); }

  void beginTaskS(uint32_t id){ beginTask(s_, id, 30000); }
  void endTaskS(){ endTask(s_); }

  void report() const {
    Serial.println("----REPORT----");
    reportOne("A", a_);
    reportOne("B", b_);
    reportOne("AGG", agg_);
    reportOne("C", c_);
    reportOne("D", d_);
    reportOne("S", s_);
  }

  void printFinalReport() const {
    Serial.println("FINAL REPORT");
    report();
  }

private:
  TaskStats a_, b_, agg_, c_, d_, s_;
  uint64_t t0_ = 0;
  uint64_t periodic_us_ = 0;
  uint64_t next_report_ = 0;
  uint64_t final_report_ = 0;
  bool final_printed_ = false;

  static void resetTask(TaskStats &t){
    t = TaskStats();
  }

  static void beginTask(TaskStats &t, uint32_t id, uint64_t period){
    t.active = true;
    t.release_us = micros();
    t.deadline_us = t.release_us + period;
    t.start_us = micros();
  }

  static void endTask(TaskStats &t){
    uint64_t end = micros();
    uint64_t exec = end - t.start_us;

    if(exec > t.max_exec_us) t.max_exec_us = exec;

    if(end > t.deadline_us){
      t.misses++;
      int64_t late = end - t.deadline_us;
      if(late > t.worst_lateness_us) t.worst_lateness_us = late;
    }

    t.jobs++;
    t.active = false;
  }

  static void reportOne(const char *name, const TaskStats &t){
    Serial.printf("%s jobs=%lu misses=%lu max=%llu late=%lld\n",
      name, t.jobs, t.misses, t.max_exec_us, t.worst_lateness_us);
  }
};

TimingMonitor g_monitor;



/*****************************************************************************
-------------------------------===<Setup>===----------------------------------
*****************************************************************************/
void setup() {
  Serial.begin(115200);  //begining serial. High baud rate selected so that serial communication within tasks is executed quickly.

  pinMode(SYNC, INPUT);  //Setting inputs
  pinMode(IN_A, INPUT);
  pinMode(IN_B, INPUT);
  pinMode(IN_S, INPUT);
  pinMode(IN_MODE, INPUT);

  pinMode(ACK_A, OUTPUT);  //Setting outputs
  pinMode(ACK_B, OUTPUT);
  pinMode(ACK_AGG, OUTPUT);
  pinMode(ACK_C, OUTPUT);
  pinMode(ACK_D, OUTPUT);
  pinMode(ACK_S, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(IN_A), In_A_Pulse, RISING);  // creating interrupts for counting A and B pulses
  attachInterrupt(digitalPinToInterrupt(IN_B), In_B_Pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(IN_S), In_S_Pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(SYNC), In_SYNC_Pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(IN_MODE), In_MODE_Pulse, RISING);

  syncSemaphore = xSemaphoreCreateBinary(); //the kernal will reserve a chunk of RAM dedicated to this semaphore. The memory chunk in the RAM hold things like the state of the semaphore and the list of tasks waiting for it.
  sSemaphore = xSemaphoreCreateBinary();  //the address of the chuck reserved in memory is stored at
  xTokenMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskA,"TaskA",4096,NULL,6,NULL,app_cpu);  //Task setups
  xTaskCreatePinnedToCore(TaskB,"TaskB",4096,NULL,5,NULL,app_cpu);
  xTaskCreatePinnedToCore(TaskAGG,"TaskAGG",4096,NULL,4,NULL,app_cpu);
  xTaskCreatePinnedToCore(TaskC,"TaskC",4096,NULL,3,NULL,app_cpu);
  xTaskCreatePinnedToCore(TaskD,"TaskD",4096,NULL,2,NULL,app_cpu);
  xTaskCreatePinnedToCore(TaskS,"TaskS",4096,NULL,1,NULL,app_cpu);

  g_monitor.setPeriodicReportEverySeconds(1);
  g_monitor.setFinalReportAfterSeconds(FINAL_REPORT_AFTER_SECONDS);
}



/*****************************************************************************
--------------------------------===<Main>===----------------------------------
*****************************************************************************/
void loop() {
  g_monitor.pollReports();
  delay(10);
}



/*****************************************************************************
----------------------------===<Work kernel>===-------------------------------
*****************************************************************************/
static inline uint32_t wk_get_cycle32() {
  uint32_t c; asm volatile("rsr %0, ccount":"=a"(c)); return c;
}
#define COMPILER_BARRIER() asm volatile("" ::: "memory")

static inline uint32_t mix32(uint32_t x){
  x ^= x >> 16; x *= 0x7FEB352D;
  x ^= x >> 15; x *= 0x846CA68B;
  x ^= x >> 16; return x;
}

static uint32_t WorkKernel(uint32_t budget, uint32_t seed){
  uint32_t start = wk_get_cycle32();
  uint32_t acc = seed;

  while((uint32_t)(wk_get_cycle32() - start) < budget){
    acc = mix32(acc);
    COMPILER_BARRIER();
  }
  return acc;
}



/*****************************************************************************
--------------------------------===<ISRs>===----------------------------------
*****************************************************************************/
void In_A_Pulse() 
{
  ++countA; //incroment countA for TaskA
}

void In_B_Pulse() 
{
  ++countB; //incroment countB for TaskB
}

void In_S_Pulse() 
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void In_SYNC_Pulse()
{
  xSemaphoreGiveFromISR(syncSemaphore, NULL);
}

void In_MODE_Pulse()
{
  modeFlag = !modeFlag; //toggle modeFlag, as it is checked using an if statement in the tasks to toggle C and D ON/OFF
}


/*****************************************************************************
-------------------------------===<Tasks>===----------------------------------
*****************************************************************************/
void TaskA(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY); //wait for the SYNC pulse to start the whole system 
  xSemaphoreGive(syncSemaphore); //re-give so other tasks waiting for SYNC can also stary
  TickType_t xLastWakeTime = xTaskGetTickCount(); //Takes mutex so that countA cant be cant be interfeared with
  
  while(1)
  {
    if (xSemaphoreTake(xTokenMutex, portMAX_DELAY) == pdTRUE)
	  {
      g_monitor.beginTaskA(IDA);
      digitalWrite(ACK_A, HIGH);
      uint16_t countA_total = countA;
      countA = 0;
      uint32_t seed = (IDA << 16) ^ countA_total ^ 0xA1;
      tokenA = WorkKernel(672000, seed);
      xSemaphoreGive(xTokenMutex);  //returns mutex allowing countA read/writes
      Serial.printf("A,%d,%d,%d\n",IDA,countA_total,tokenA);
      ++IDA;
      digitalWrite(ACK_A, LOW);
      g_monitor.endTaskA();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // Exact 10ms period
  }
}

void TaskB(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  xSemaphoreGive(syncSemaphore);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    if (xSemaphoreTake(xTokenMutex, portMAX_DELAY) == pdTRUE)
	  {
      g_monitor.beginTaskB(IDB);
      digitalWrite(ACK_B, HIGH);
      uint16_t countB_total = countB;
      countB = 0;
      uint32_t seed = (IDB << 16) ^ countB_total ^ 0xB2;
      tokenB = WorkKernel(960000, seed);
      xSemaphoreGive(xTokenMutex);
      Serial.printf("B,%d,%d,%d\n",IDB,countB_total,tokenB);
      ++IDB;
      digitalWrite(ACK_B, LOW);
      g_monitor.endTaskB();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // Exact 20ms period
  }
}

void TaskAGG(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  xSemaphoreGive(syncSemaphore);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    if (xSemaphoreTake(xTokenMutex, portMAX_DELAY) == pdTRUE)
    {
      g_monitor.beginTaskAGG(IDAGG);
      digitalWrite(ACK_AGG, HIGH);
      uint16_t AGG = tokenA ^ tokenB;
      xSemaphoreGive(xTokenMutex);
      uint32_t seed = (IDAGG << 16) ^ AGG ^ 0xD4;
      uint32_t tokenAGG = WorkKernel(480000, seed);
      Serial.printf("AGG,%d,%d,%d\n",IDAGG,AGG,tokenAGG);
      ++IDAGG;
      digitalWrite(ACK_AGG, LOW);
      g_monitor.endTaskAGG();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // Exact 20ms period
  }
}

void TaskC(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  xSemaphoreGive(syncSemaphore);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    if(modeFlag)
    {
      g_monitor.beginTaskC(IDC);
      digitalWrite(ACK_C, HIGH);
      uint32_t seed = (IDC << 16) ^ 0xC3;
      uint32_t tokenC = WorkKernel(1680000, seed);
      Serial.printf("C,%d,%d\n",IDC,tokenC);
      ++IDC;
      digitalWrite(ACK_C, LOW);
      g_monitor.endTaskC();
    }
    else
    {
      Serial.printf("Mode LOW C\n");
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // Exact 50ms period
  }
}

void TaskD(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  xSemaphoreGive(syncSemaphore);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    if(modeFlag)
    {
      g_monitor.beginTaskD(IDD);
      digitalWrite(ACK_D, HIGH);
      uint32_t seed = (IDD << 16) ^ 0xD5;
      uint32_t tokenD = WorkKernel(960000, seed);
      Serial.printf("D,%d,%d\n",IDD,tokenD);
      ++IDD;
      digitalWrite(ACK_D, LOW);
      g_monitor.endTaskD();
    }
    else
    {
      Serial.printf("Mode LOW D\n");
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // Exact 50ms period
  }
}

void TaskS(void *pvParameters) 
{
  xSemaphoreTake(syncSemaphore, portMAX_DELAY);
  xSemaphoreGive(syncSemaphore);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    if (xSemaphoreTake(sSemaphore, portMAX_DELAY) == pdTRUE)
    {
      g_monitor.beginTaskS(IDS);
      digitalWrite(ACK_S, HIGH);
      uint32_t seed = (IDS << 16) ^ 0x55;
      uint32_t tokenS = WorkKernel(600000, seed);
      Serial.printf("S,%d,%d\n",IDS,tokenS);
      ++IDS;
      digitalWrite(ACK_S, LOW);
      g_monitor.endTaskS();
    }
  }
}



