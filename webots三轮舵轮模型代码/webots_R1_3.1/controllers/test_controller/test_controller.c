/*
 * File:          control.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/joystick.h>
#include <math.h>
#include <stdio.h>
#include <webots/utils/default_robot_window.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define len_car 0.75
#define wid_car 0.75
#define A_car 0.338
#define PI 3.14159
#define Half_pi (PI/2)
#define Two_pi (PI*2)

double V_car;
double R_car;
float a;
float b;
float angle_car;
float angle_LB;
float angle_RB;
float angle_LF;
float angle_RF;

float speed_LB;
float speed_RB;
float speed_LF;
float speed_RF;
double Vel_Y = 0;
double Vel_X = 0;
double Vel_W = 0;
WbDeviceTag motors[8];
WbDeviceTag sensor[4];


char motors_names[6][40] = {"Motor_F_Move", "Motor_LB_Move","Motor_RB_Move", "Motor_F_Turn","Motor_LB_Turn", "Motor_RB_Turn"};
 char sensor_names[3][40] = {"Motor_F_Senor","Motor_LB_Senor","Motor_RB_Senor"};

typedef struct FourSteering_Move_TypeDef
{
	float Vel_Y;
	float Vel_X;
	float Vel_W;
	
	float Motor_Tar[8];	
}FourSteering_Move_TypeDef;

typedef struct Motor_3510_TypeDef
{
	float Tar_Vel;
	float Tar_Pos;
}Motor_3510_TypeDef;
typedef struct Motor_3508_TypeDef
{
	float Tar_Vel;
	float Tar_Pos;
}Motor_3508_TypeDef;
typedef struct Chassis_Control
{
	Motor_3510_TypeDef Motor_Turn[4];
	Motor_3508_TypeDef Motor_Move[4];
}Chassis_Control;

typedef struct Chassis_TypeDef
{
	FourSteering_Move_TypeDef Chassis_Motor_Tar;
	float Motor_Pos[4];
	Chassis_Control Chassis_Control_Motor;
	
}Chassis_TypeDef;
Chassis_TypeDef Chassis;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
void FourSteering_Chassis_IK_Cal(float Vel_Y,float Vel_X,float Vel_W)
{
	V_car = sqrt(Vel_X*Vel_X + Vel_Y*Vel_Y);
	angle_car = atan2(Vel_X , Vel_Y) ;
	b = R_car * sin(angle_car);
	a = R_car * cos(angle_car);
	
	
            if(Vel_W == 0 && (Vel_Y != 0 || Vel_X != 0))
	{
            if(Vel_X != 0 && Vel_Y != 0) angle_car = atan(Vel_X / Vel_Y) ;
            else if(Vel_X != 0) angle_car = 1.57;
            else angle_car = 0;
            angle_LF = angle_car;
            angle_LB = angle_car;
            angle_RF = angle_car;
	speed_LF = V_car;
	speed_LB = V_car;
	speed_RF = V_car;
	}
	else
	{
	R_car = V_car / Vel_W;
	// angle_LF = -atan2((b + len_car / 2) , (a + wid_car / 2)  );
	// angle_LB = -atan2((b - len_car / 2) , (a + wid_car / 2)  );
	// angle_RF = -atan2((b + len_car / 2) , (a - wid_car / 2)  );
	// angle_RB = -atan2((b - len_car / 2) , (a - wid_car / 2)  );
	angle_LF = -atan2(b + A_car, a );
	angle_LB = -atan2((b - A_car / 2) , (a + A_car * 1.7f / 2)  );
	angle_RF = -atan2((b + A_car / 2) , (a + A_car * 1.7f / 2)  );
	speed_LF = sqrt((b + A_car) * (b + A_car) + a * a) / R_car * V_car;
	speed_LB = sqrt((b - A_car / 2) * (b - A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * V_car;
	speed_RF = sqrt((b + A_car / 2) * (b + A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * V_car;
	
	}
	//printf("%.2f %.2f %.2f\n",angle_LF,angle_LB,speed_RF);

	Chassis.Chassis_Motor_Tar.Motor_Tar[0] = speed_LF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[1] = speed_LB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[2] = speed_RF;
	
	Chassis.Chassis_Motor_Tar.Motor_Tar[3] = angle_LF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[4] = angle_LB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[5] = angle_RF;
}

void Motor_Angle()
{

	for (int num = 0;num < 3;num++)
	{
		if (Chassis.Motor_Pos[num] > 0)
		{
		//printf("%.2f %.2f %.2f %.2f\n",Chassis.Chassis_Motor_Tar.Motor_Tar[4],Chassis.Chassis_Motor_Tar.Motor_Tar[5],Chassis.Chassis_Motor_Tar.Motor_Tar[6],Chassis.Chassis_Motor_Tar.Motor_Tar[7]);
		   //printf("%d %.2f %.2f\n",num,Chassis.Motor_Pos[num],Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]);
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] <= Half_pi) || (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) >= -Half_pi)
			{
				//printf("MA %.2f %.2f %.2f\n",angle_LF,angle_LB,angle_RF);
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+3];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos= Chassis.Motor_Pos[num] + Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num];
			}
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > PI*3/4)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] += 2*PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]);
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] += PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (-(Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]));
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] -= PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (-(Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]));
			} 
		}
		else if (Chassis.Motor_Pos[num] < 0)
		{
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) || (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+4];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]);
			}
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < -PI*3/4)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] -= 2*PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]);
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] += PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]);
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] -= PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (-(Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]));
			} 
		}
		else
		{
          		  printf("error");
		}
			
	}
}

void Motor_Speed()
{
	for (int num = 0;num < 3;num++)
	{
		if (Chassis.Motor_Pos[num] > 0)
		{
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) || (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi || Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > PI*3/4)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else 
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
		}
		else if (Chassis.Motor_Pos[num] < 0)
		{
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) || (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi || Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < -PI*3/4)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else 
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
		}
			
	}
}
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   wbu_default_robot_window_configure();
  for (int i = 0; i < 6; i++) {
    motors[i] = wb_robot_get_device(motors_names[i]);

  }
  for (int i = 0;i < 3;i++)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i],0);
    sensor[i] = wb_robot_get_device(sensor_names[i]);
    wb_position_sensor_enable(sensor[i],TIME_STEP);
  }wb_joystick_enable(TIME_STEP);



  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
   //Vel_Y = (float)wb_joystick_get_axis_value(0) / 35536.0f * 20;
   Vel_X = (float)wb_joystick_get_axis_value(1) / 35536.0f * 20;
    if(Vel_X < 1 && Vel_Y > -1) Vel_X = 0;
    printf("%.2f %.2f %.2f\n",Vel_Y,Vel_X,Vel_W);
    /* Process sensor data here */
    for (int num = 0;num < 3;num++)
    {
      Chassis.Motor_Pos[num] = wb_position_sensor_get_motor(sensor[num]);
      //printf("%.2f\n",Chassis.Motor_Pos[num]);
    }
    
   FourSteering_Chassis_IK_Cal(Vel_Y,Vel_X,Vel_W);
   Motor_Angle();
   Motor_Speed();

    wb_motor_set_velocity(motors[0],Chassis.Chassis_Control_Motor.Motor_Move[0].Tar_Vel );
    wb_motor_set_velocity(motors[1],Chassis.Chassis_Control_Motor.Motor_Move[1].Tar_Vel );
    wb_motor_set_velocity(motors[2],Chassis.Chassis_Control_Motor.Motor_Move[2].Tar_Vel );
    wb_motor_set_position(motors[3],Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Pos);
    wb_motor_set_position(motors[4],Chassis.Chassis_Control_Motor.Motor_Turn[1].Tar_Pos);
    wb_motor_set_position(motors[5],Chassis.Chassis_Control_Motor.Motor_Turn[2].Tar_Pos);
    //printf("%.2f %.2f %.2f\n",Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Pos,Chassis.Chassis_Control_Motor.Motor_Turn[1].Tar_Pos,Chassis.Chassis_Control_Motor.Motor_Turn[2].Tar_Pos);
    //printf("OK1 %.2f\n",Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Vel);

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     wbu_default_robot_window_set_images_max_size(100,100);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

