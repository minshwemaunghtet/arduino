#include <NewPing.h>  // Library for ultrasonic sensor

#define AVERAGE_FRAME_COUNT 100

enum DEVICE_STATE
{
  STATE_REST = 0,
  STATE_PIPE_DETECTED = 1,
  STATE_PIPE_READY = 2,
  STATE_PIPE_FINISHED = 3,
};

DEVICE_STATE device_state = STATE_REST;
const int TRIG_PIN = 7; 
const int ECHO_PIN = 6; 
const int LED_PIN  = 3;
NewPing sonar(TRIG_PIN, ECHO_PIN);  //Max to ping in cm. Max sensor distance is rated at 400-500cm.

int g_baseDistance = 0;    // distance when no pipe is present
int g_currentDistance = 0; // current measured distance
int g_avgDistance = 0;     // average distance over last 25 frames
int g_frame       = 0;     // current loop
int g_distances[AVERAGE_FRAME_COUNT];  // ? g_distances[100]
int g_distanceMove = 0;


void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);  
  pinMode(LED_PIN, OUTPUT);

  g_baseDistance = sonar.ping_cm();
  g_currentDistance = g_baseDistance;
  g_avgDistance = g_baseDistance;
  Serial.print("Base Distance: ");
  Serial.println(g_baseDistance);

  for(int i = 0; i < AVERAGE_FRAME_COUNT; i++)
  {
    g_distances[i] = g_baseDistance;    
  }
}

bool restState()  // This function initially calculate distance from the wall
{
  // if pipe is detected, return true else return false
  if (g_currentDistance <= g_baseDistance-5)  // Detect can be only based on distance. Suppose pipe is within 30 cm.
  {                                           // Current distance will get from first base distance
    return true; 
  }
  else
  {
    return false;
  }
}

bool triggerState()  // This function knows to detect object at some random distance
{
  if(g_currentDistance == 0)
    return false;
  if(g_currentDistance > g_avgDistance - 5 && g_currentDistance < g_avgDistance + 5 && g_avgDistance < g_baseDistance-5)
  {
    return true;
  }
  return false;
  
  // if pipe stopped, return true else return false
  bool IS_STOP;
  if(sonar.check_timer())  // Check if ping has returned within the set distance limit.
  {
    IS_STOP = true;
  }
  else
  {
    IS_STOP = false;
  }

  if (IS_STOP == true)
  {
      long duration;
      int distance;
      int max_distance = 30; // 30cm
      int min_distacne = 0;
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH);  // Echo will get the signal back from Trigm which has sent
      distance= (duration/2)/29.1;  // Convert in cm
      if (distance >= max_distance || distance <= min_distacne)  // check object is in range or not
      {
        Serial.println("It cannot detect object!");
        
      }
      else
      {
        digitalWrite(LED_PIN, OUTPUT);
      }
  }
  else
  {
  return false;
  }
}

void finalState()  // This function will blink the LED
{
  int distance;
  int obj_position;
  if (distance == obj_position)  // Example code, can't run
  {
    //digitalWrite(LED_PIN, OUTPUT);
  }
  
}

bool waitingOnPipeToMove()
{
  // when moved, return true;
  int g_distanceMove = g_baseDistance; // To store the latest distance
  if (g_distanceMove > g_currentDistance || g_distanceMove < g_currentDistance)
  {
    Serial.print(g_distanceMove);
    Serial.print("\t");
    Serial.println(g_baseDistance);
    return true;
  }
  else
  {
    return false;
  }
  
}




// Main loop
void loop() 
{
  delay(16);  // To run slower because inside loop
  g_currentDistance = sonar.ping_cm();
  g_distances[g_frame] = g_currentDistance;  // g_distances[0]
  int sum = 0;
  // get average
  for(int i = 0; i < AVERAGE_FRAME_COUNT; i++)
  {
    sum += g_distances[i];
  }
  g_avgDistance = sum/AVERAGE_FRAME_COUNT;
  g_frame++;

  // reset
  if(g_frame >= AVERAGE_FRAME_COUNT)
    g_frame = 0;
  
  if(device_state == STATE_REST)
  {
    if(restState()) // try to detect pipe
      device_state = STATE_PIPE_DETECTED;
  }

  else if(device_state == STATE_PIPE_DETECTED)
  {

    if(triggerState()) // try to detect pipe stopping
      device_state = STATE_PIPE_READY;
  }

  else if(device_state == STATE_PIPE_READY)
  {

    finalState();
    Serial.println("Pipe stopped and scanned. Reset");
    Serial.print("Base Distance: ");
    Serial.println(g_baseDistance);
    digitalWrite(LED_PIN, OUTPUT);

    for(int i = 0; i < AVERAGE_FRAME_COUNT; i++)
    {
      g_distances[i] = g_baseDistance;    
    }

    device_state = STATE_PIPE_FINISHED;
  }
  
  else if(device_state == STATE_PIPE_FINISHED)
  {

    if(waitingOnPipeToMove())
    {
      device_state = STATE_REST;
    }

  }
  






// Leave this for now!
//  unsigned long interval;
//  interval = millis();

//  if (duration < 1000)             //How to check interval of the object
//  {
//    if (blink_state == LOW)
//    {
//      blink_state = HIGH;
//      digitalWrite(LED_PIN, blink_state);
//    }
//  }
//  else
//    {
//      blink_state = LOW;
//      digitalWrite(LED_PIN, blink_state);
//    }

    

}
