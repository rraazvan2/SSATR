#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h>

QueueHandle_t queue_1;
TaskHandle_t task1_handle = NULL, task2_handle = NULL, task3_handle = NULL, task4_handle = NULL;
Servo myservo;

typedef signed int sint;
typedef unsigned int uint;

void TaskAnalogRead(void *pvParameters);  // Prototype for Task 0
void TaskControlServoMotor(void *pvParameters);  // Prototype for Task 1
void TaskDisplayValue(void *pvParameters);  // Prototype for Task 2
void TaskManualControl(void *pvParameters);  // Prototype for Task 3

  uint Temperature_senzor_pin = A0; // Temperature sensor is connected to analog pin A0
  uint current_intensity; // Value of sensor
  float voltage; // Voltage in volts
  float temperature; // Temperature in degrees Celsius

  uint Potentiometer_pin = A1; // Potentiometer is connected to analog pin A1
  uint Value_of_potentiometer; // Value of potentiometer
  uint Servo_pin = 9; // Servo is connected to digital pin 9
  uint Servo_poz = 0; // Servo position

  uint External_interrupt_pin = 2; // External interrupt is connected to digital pin 2
  volatile byte state = LOW; // State of external interrupt

#define LED_pin LED_BUILTIN // LED is connected to digital pin 13

void setup()
{
  // Initialize serial comunication at 9600 bits per second:
  Serial.begin(9600);

  // Create a queue capable of containing 5 unsigned long values.
  queue_1 = xQueueCreate( 5, sizeof( unsigned long ) );
  if ( queue_1 == NULL )
  {

    Serial.println(F("Queue creation failed"));

  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskAnalogRead
    , (const portCHAR *)"AnalogRead"   // A name just for humans
    , 128  // This stack size can be checked & adjusted by reading the Stack Highwater
    , NULL // task parameters,
    , 0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    , task1_handle // task handle
    );

  xTaskCreate(
    TaskControlServoMotor
    ,  (const portCHAR *)"ControlServoMotor"   // A name just for humans
    ,  256  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  task2_handle 
    );
  
  xTaskCreate(
    TaskDisplayValue
    ,  (const portCHAR *)"DisplayValue"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  task3_handle 
    );

  xTaskCreate(
    TaskManualControl
    ,  (const portCHAR *)"ManualControl"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  task4_handle 
    );

  vTaskStartScheduler();  // This function never returns unless RTOS scheduler runs out of memory and fails
  
  myservo.attach(Servo_pin);  // attaches the servo on pin 9 to the servo object
  pinMode(External_interrupt_pin, INPUT_PULLUP); // Set pin 2 as input
  attachInterrupt(digitalPinToInterrupt(External_interrupt_pin), MODE_AM, CHANGE); // Set external interrupt on pin 2
  pinMode(LED_pin, OUTPUT); // Set pin 13 as output

}

void MODE_AM()
{
  state = !state;
}

void loop()
{
  // DO nothing
}

void TaskAnalogRead( void *pvParameters ) // This is a Task for reading value of sensor.
{
  for(;;)
  {

    current_intensity = analogRead(Temperature_senzor_pin);
    voltage = current_intensity * (5.0 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    temperature = voltage - 0.5;
    temperature = temperature / 0.01; // Convert the voltage to temperature in degrees Celsius
    xQueueSend(queue_1, &temperature, portMAX_DELAY);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );  // one tick delay (15ms) in between reads for stability

  }

}

void TaskDisplayValue( void *pvParameters ) // This is a Task for display value of sensor.
{
  for(;;)
  {
    xQueueReceive(queue_1, &temperature, portMAX_DELAY);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    vTaskDelay( 1000 / portTICK_PERIOD_MS );  // one tick delay (15ms) in between reads for stability
    
  }

}

void TaskControlServoMotor( void *pvParameters ) // This is a Task for control servo motor.
{
  for(;;)
  {

    if (state)
    {

      //digitalWrite(LED_pin, LOW); // Turn LED on
      vTaskSuspend(task4_handle);

    }
    else
    {

      //digitalWrite(LED_pin, HIGH); // Turn LED off
      vTaskResume(task4_handle);

    }
    xQueueReceive(queue_1, &temperature, portMAX_DELAY);
    
    if((20 <= temperature) && (temperature >= 25.0))
    {
      myservo.write(90);
    }
    else
    {
      
        if(25.0 < temperature)
        {
          
          for(Servo_poz = 90; Servo_poz <= (90+(temperature - 25.0));Servo_poz++)
          {
            myservo.write(Servo_poz);
          }

        }
        else
        {
          for(Servo_poz = 90; (uint)Servo_poz >= (90-(20.0 - temperature));Servo_poz--)
          {
            myservo.write(Servo_poz);
          }
        }

    }
    vTaskDelay( (uint)1000 / portTICK_PERIOD_MS );  // one tick delay (15ms) in between reads for stability
  }
}

void TaskManualControl( void *pvParameters ) // This is a Task for manual control servo motor.
{
  Serial.println("Manual control activated");
  for(;;)
  {

    if(state)
    {
      vTaskSuspend(task1_handle);
      vTaskSuspend(task2_handle);
      vTaskSuspend(task3_handle);
      Value_of_potentiometer = analogRead(Potentiometer_pin);
    Value_of_potentiometer = Value_of_potentiometer * (180.0 / 1023.0);
    myservo.write((uint)Value_of_potentiometer);
    }
    else
    {
      xTaskResumeAll();
      vTaskSuspend(NULL);
      Serial.println("Automatic control activated");
    }
    vTaskDelay( (uint)1000 / portTICK_PERIOD_MS );  // one tick delay (15ms) in between reads for stability
  }
}
