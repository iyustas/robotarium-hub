/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  Robot.ino
 * RUTA:     /firmware/Robot.ino
 * REPO:     https://github.com/UCM-237/robotarium-hub
 * ----------------------------------------------------------------------------
*/


#include "common.h"
#include "controler.h"
#include "robot.h"
#include <SimpleKalmanFilter.h>
#include <math.h>
// --- CONFIGURACIÓN DEL TEST ---
double VELOCIDAD_OBJETIVO = 0.01; // rad/s (ajusta según necesites)
bool TEST_RUEDA_DERECHA =true;   // true para derecha, false para izquierda
// ------------------------------

MeanFilter<long> meanFilterRight(10);
MeanFilter<long> meanFilterLeft(10);
robot miRobot;
controler PID_Rueda;
  int imprime=0;
  int contador=0;
  int pwm_output=150; 
void setup() {
  Serial.begin(115200);
  miRobot.pinSetup();
  miRobot.motorSetup();
  double A=11;
  double B=44;
  // Paso 1: Objetivo realista (aprox 2 vueltas por segundo)
  VELOCIDAD_OBJETIVO =15; 
  PID_Rueda.setSetPoint(VELOCIDAD_OBJETIVO);
  PID_Rueda.setFeedForwardParam(11,44);

  // Paso 2: Solo proporcional (Kp). Ki y Kd a CERO.
  // Un Kp de 2.0 o 5.0 es un buen inicio para motores de bajo coste.
  PID_Rueda.setControlerParam(1, 0.0,0.0);
  // Configuración de interrupciones para encoders (necesario para calcular w real)
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isr_right, RISING);

  Serial.println("Objetivo(rad/s),Real(rad/s),PWM");

                                                                              //mod
}

void loop() {
 //tiempo para que los tiempos del
  static unsigned long segundoMillis = 0;
  double fD, fI;
  timeStopD=timeStopI=millis();
  meanFilterRight.AddValue(deltaTimeRight);
  meanFilterLeft.AddValue(deltaTimeLeft);

  deltaTimeStopD=timeStopD-timeAfterDebounceRight;
  deltaTimeStopI=timeStopI-timeAfterDebounceLeft;

if (millis() - segundoMillis >= 10000) {
    segundoMillis = millis(); // Espera 1000 milisegundos = 3 segundos
     // contador ++ ;
    //if (pwm_output >= 250){
  //    pwm_output = 250;
    //};
    //if (contador ==7){
    //pwm_output =pwm_output +50;
    //imprime=1;
//contador=0;
}
//if (contador ==7){

//contador=0;
//};
  static unsigned long lastMillis = 0;
   // int pwm_output=0;                                                                           //mod
      
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 10) {
    lastMillis = millis();

    double w_real;
    long time_filtered;
    // Calcular velocidad real (rad/s) basándonos en los encoders
    if (TEST_RUEDA_DERECHA) {
      // Cálculo simplificado basado en el tiempo entre pulsos (deltaTimeRight)
      // Nota: Asegúrate de que deltaTimeRight se actualice en la ISR
      time_filtered=meanFilterRight.GetFiltered();
      fD=deltaTimeStopD>=100 ? 0 : (double)1/(meanFilterRight.GetFiltered()*MAX_ENCODER_STEPS)*1000;
      w_real = (3.35*PI*fD)/10.0; 
      //if (millis() - timeAfterRight > 100) w_real = 0; // Si no hay pulsos, velocidad 0

// 1. Calculamos la base necesaria para la velocidad objetivo (FF)
int base_pwm = PID_Rueda.feedForward(); 

// 2. Calculamos el ajuste fino (PID)
int ajuste_pid = PID_Rueda.pid(w_real);

// 3. Sumamos ambos. Si w_real < deseada, ajuste_pid será positivo y subirá de la base.

//pwm_output = constrain(base_pwm , 100, 255);                                                   //mod
//  Serial.print("                base_pwm:");
//  Serial.println(base_pwm );
  
miRobot.moveRightWheel(pwm_output, VELOCIDAD_OBJETIVO, false);    }  
 
                              
    else {
      fI=deltaTimeStopI>=100 ? 0 : (double)1/(meanFilterLeft.GetFiltered()*MAX_ENCODER_STEPS)*1000;
      w_real = (3.35*PI*fI)/10.0; 
     // if (mills() - timeAfterLeft > 100) w_real = 0;

    //Serial.print(pwm_output); // O el PWM si quieres verlo                                         //quita 
    //Serial.print("               ");                                                               //quita

     // pwm_output = PID_Rueda.pid(w_real);                                                        //descomenta
      miRobot.moveLeftWheel(pwm_output, VELOCIDAD_OBJETIVO, true);
    }
    // Formato para el Serial Plotter de Arduino
   // Serial.print(VELOCIDAD_OBJETIVO);
    //Serial.print("               ");
    //if (imprime==1){
    //imprime=0;
    //}                                                                                 //quita
    //Serial.println(encoder_countLeft);
  
    Serial.print(w_real);
    Serial.print("               ");
    Serial.print(pwm_output); // O el PWM si quieres verlo
    Serial.print("               ");
    Serial.println(time_filtered);
  }
}
/*
// --- ISR Necesarias (Copiadas de tu lógica en Robot.ino) ---
void isr_right() {
unsigned long ahora = micros();
  // Bajamos el debounce a 1000 microsegundos (1ms)
  if (ahora - timeBeforeDebounceRight > 1000) { 
    deltaTimeRight = ahora - startTimeRight;
    startTimeRight = ahora;
    timeBeforeDebounceRight = ahora;
    timeAfterRight = ahora; // Usado para detectar si el robot se detuvo
    encoder_countRight++;
    
  }
  }

void isr_left() {
  timeAfterLeft = micros();
  deltaTimeLeft = timeAfterLeft - startTimeLeft;
  startTimeLeft = timeAfterLeft;
  encoder_countLeft++;
}
*/

void isr_right() {
  // 1. GESTIÓN DE REBOTES (Debouncing)
  timeBeforeDebounceRight = millis(); // Captura el tiempo actual del pulso
  deltaDebounceRight = timeBeforeDebounceRight - timeAfterDebounceRight; // Tiempo desde el último pulso (aunque fuera ruido)

  // Solo procesamos el pulso si ha pasado suficiente tiempo (TIMEDEBOUNCE)
  // Esto filtra picos de voltaje o vibraciones mecánicas que darían velocidades falsas
  if(deltaDebounceRight > TIMEDEBOUNCE) {
   
    // 2. CONTEO DE ODOMETRÍA
    startTimeRight = millis(); // Marca el tiempo de inicio de este pulso "válido"
    encoder_countRight++;      // Incrementa el contador total de pasos
   
    // Si llegamos al número de pasos por vuelta, incrementamos el contador de vueltas completas
    if(encoder_countRight == MAX_ENCODER_STEPS) {
      wheelTurnCounterRight++;
    }

    // 3. CÁLCULO DE VELOCIDAD (Delta Time)
    // Calcula el tiempo exacto transcurrido entre este pulso y el anterior
    // Este valor es inversamente proporcional a la velocidad de la rueda
    deltaTimeRight = startTimeRight - timeAfterRight;
     
    // Guarda este instante para compararlo con el próximo pulso
    timeAfterRight = startTimeRight;
  }
 
  // Actualiza el registro de tiempo para el siguiente chequeo de rebote
  timeAfterDebounceRight = timeBeforeDebounceRight;  
}

/**
 * ISR para la rueda izquierda.
 * Realiza la misma lógica de filtrado y conteo para el motor izquierdo.
 */
void isr_left() {
  timeBeforeDebounceLeft = millis();
  deltaDebounceLeft = timeBeforeDebounceLeft - timeAfterDebounceLeft;

  if(deltaDebounceLeft > TIMEDEBOUNCE) {
    startTimeLeft = millis();
    encoder_countLeft++;

    if(encoder_countLeft == MAX_ENCODER_STEPS) {
      wheelTurnCounterLeft++;
    }
   // 3. CÁLCULO DE VELOCIDAD (Delta Time)
    // Calcula el tiempo exacto transcurrido entre este pulso y el anterior
    // Este valor es inversamente proporcional a la velocidad de la rueda
    deltaTimeLeft = startTimeLeft - timeAfterLeft;
     
    // Guarda este instante para compararlo con el próximo pulso
    timeAfterLeft = startTimeLeft;
  }
 
  // Actualiza el registro de tiempo para el siguiente chequeo de rebote
  timeAfterDebounceLeft = timeBeforeDebounceLeft;  

}