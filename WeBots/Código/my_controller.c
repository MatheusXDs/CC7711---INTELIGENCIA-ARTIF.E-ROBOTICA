#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define TIME_STEP 64

//calcula velocidade do robo normalizada
float normalizaVetor(float x, float y){
  return sqrt( (x*x) + (y*y)) ;
}

//funcoes para movimentar e girar o robo
void vira_direita(WbDeviceTag MotorEsquerdo, WbDeviceTag MotorDireito ){
  wb_motor_set_velocity(MotorEsquerdo, 6.2);
  wb_motor_set_velocity(MotorDireito, -6.2);
}
void vira_esquerda(WbDeviceTag MotorEsquerdo, WbDeviceTag MotorDireito ){
  wb_motor_set_velocity(MotorDireito, 6.2);
  wb_motor_set_velocity(MotorEsquerdo, -6.2);
}
void para_frente(WbDeviceTag MotorEsquerdo, WbDeviceTag MotorDireito ){
  wb_motor_set_velocity(MotorDireito, 6.2);
  wb_motor_set_velocity(MotorEsquerdo, 6.2);

}
void motor_stop(WbDeviceTag MotorEsquerdo, WbDeviceTag MotorDireito ) {
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);
}

//sinaliza quais sensores de distancia estao ativos
bool * get_sensor_ativos(WbDeviceTag *proximitySensors)
{
  static bool sensors_condition[8] = {false};
	
  for (int i = 0; i < 8 ; i++) 
  {
    if (wb_distance_sensor_get_value(proximitySensors[i]) > 400) 
    {sensors_condition[i] = true;
      } else{sensors_condition[i] = false;}

  }
  	
  return sensors_condition;
}


int main(int argc, char **argv) {
  
  wb_robot_init();
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("epuck"); //captura o supervisor
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  const double *posicao;
  double posicao_anterior[2] = {0,0};
  double distancia[2] = {0, 0};
  //int temp,i,f = 0;
  //double proximitySensorValues[8];
  //bool ligado = false;
  int i;
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);
  
  WbDeviceTag proximitySensors[8];
  char names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  for (int i = 0; i < 8 ; i++) {
    proximitySensors[i] = wb_robot_get_device(names[i]);
    wb_distance_sensor_enable(proximitySensors[i], TIME_STEP);
  }
  
  
  WbDeviceTag LD[10];
  char LD_names[10][5] = {
    "led0", "led1", "led2", "led3",
    "led4", "led5", "led6", "led7",
    "led8","led9"
  };
  for (i = 0; i < 10 ; i++) {
    LD[i] = wb_robot_get_device(LD_names[i]);
    wb_led_set(LD[i], 0);
  }
  
  posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
  
  while (wb_robot_step(TIME_STEP) != -1) {
      posicao_anterior[0] = posicao[0];
      posicao_anterior[1] = posicao[2];
      posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
      distancia[0] = posicao[0] - posicao_anterior[0];
      distancia[1] = posicao[2] - posicao_anterior[1];
      
  
    bool *sensor_ativo = get_sensor_ativos(proximitySensors);
    //printf("%f\n", normalizaVetor(distancia[0], distancia[1]));
    
    if ((sensor_ativo[7] || sensor_ativo[0] || sensor_ativo[1]  || sensor_ativo[6]) && normalizaVetor(distancia[0], distancia[1]) >= 0.007000 ){
      wb_led_set(LD[0], 1);
    }

    else if (sensor_ativo[7] || sensor_ativo[6]){
    vira_direita(MotorEsquerdo, MotorDireito);
    wb_led_set(LD[0], 0);
    }
    
    else if (sensor_ativo[0] || sensor_ativo[1]){
    vira_esquerda(MotorEsquerdo, MotorDireito);
    wb_led_set(LD[0], 0);
    }
    
    
    
    else {
    para_frente(MotorEsquerdo, MotorDireito);
    }
    
    
  };

  wb_robot_cleanup();

  return 0;
}