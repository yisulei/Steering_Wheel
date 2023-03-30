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
#include <math.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define len_car 0.75
#define wid_car 0.75
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
double Vel = 1;
double Vel_Y = 0;
double Vel_X = 0;
double Vel_W = 0;
int mode = 0;
float pos = 0;
int flag1 = 1;
int flag2 = 1;
WbDeviceTag motors[8];
WbDeviceTag sensor[4];
WbDeviceTag compass;
WbDeviceTag IMU;

char motors_names[8][40] = {"LF_move", "LB_move","RF_move", "RB_move","LF_turn", "LB_turn","RF_turn", "RB_turn"};
char sensor_names[4][40] = {"position_sensor_LF","position_sensor_LB","position_sensor_RF","position_sensor_RB"};
   const double* angle;
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
	double Motor_Pos[4];
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
            angle_car = atan2(Vel_X , Vel_Y) ;
            angle_LF = angle_car;
            angle_LB = angle_car;
            angle_RF = angle_car;
            angle_RB = angle_car;
            if (Vel_Y >= 0)
              {
              speed_LF = V_car;
      	  speed_LB = V_car;
  	  speed_RF = V_car;
  	  speed_RB = V_car;
              }
            else
              {
               speed_LF = -V_car;
  	   speed_LB = -V_car;
  	   speed_RF = -V_car;
  	   speed_RB = -V_car;
              }
	}else if(Vel_W != 0 && (Vel_Y == 0 && Vel_X == 0))
	{
            angle_LF = -PI/4;
            angle_LB = PI/4;
            angle_RF = PI/4;
            angle_RB = -PI/4;
	 speed_LF = Vel_W;
	 speed_LB = Vel_W;
	 speed_RF = -Vel_W;
	 speed_RB = -Vel_W;
	}
	else if(Vel_W != 0 && (Vel_Y != 0 || Vel_X != 0))
	{
      	R_car = V_car / Vel_W;
      	angle_LF = -atan2((b + len_car / 2) , (a + wid_car / 2)  );
      	angle_LB = -atan2((b - len_car / 2) , (a + wid_car / 2)  );
      	angle_RF = -atan2((b + len_car / 2) , (a - wid_car / 2)  );
      	angle_RB = -atan2((b - len_car / 2) , (a - wid_car / 2)  );
      	speed_LF = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car *V_car;
      	speed_LB = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car *V_car;
      	speed_RF = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R_car *V_car;
      	speed_RB = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R_car *V_car;
	}else
	{
	speed_LF = 0;
	speed_LB = 0;
	speed_RF = 0;
	speed_RB = 0;
	
	}
	

	Chassis.Chassis_Motor_Tar.Motor_Tar[0] = speed_LF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[1] = speed_LB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[2] = speed_RF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[3] = speed_RB;
	
	Chassis.Chassis_Motor_Tar.Motor_Tar[4] = angle_LF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[5] = angle_LB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[6] = angle_RF;
	Chassis.Chassis_Motor_Tar.Motor_Tar[7] = angle_RB;
	
	
}

void IK()
{
  Vel_X = Vel*sin(angle[2]);
  Vel_Y = Vel*cos(angle[2]);
}
void Motor_Angle()
{

	for (int num = 0;num < 4;num++)
	{
	 printf("Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos=%.2f\n",Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos);
		if (Chassis.Motor_Pos[num] >= 0)
		{
		   //printf("%.2f %.2f\n",Chassis.Motor_Pos[num],Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]);
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] <= Half_pi) && (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) >= -Half_pi )
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+4];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos= Chassis.Motor_Pos[num] + Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num];
			}
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > PI*3/2)
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
			printf("**************************\n");
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] -= PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (-(Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]));
			} 
		}
		else if (Chassis.Motor_Pos[num] < 0)
		{
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) && (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+4];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] - Chassis.Motor_Pos[num]);
			}
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < -PI*3/2)
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
	for (int num = 0;num < 4;num++)
	{
		if (Chassis.Motor_Pos[num] > 0)
		{
			if (((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) && ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi ))|| Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] > PI*3/4)
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
			if (((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < Half_pi) && ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4]) > -Half_pi)) || Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+4] < -PI*3/4)
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
		printf("Motor_speed=%.2f \n",Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel);
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
   
  for (int i = 0; i < 8; i++) {
    motors[i] = wb_robot_get_device(motors_names[i]);

  }
  for (int i = 0;i < 4;i++)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i],0);
    sensor[i] = wb_robot_get_device(sensor_names[i]);
    wb_position_sensor_enable(sensor[i],TIME_STEP);
  }compass = wb_robot_get_device("compass");
  wb_compass_enable(compass,TIME_STEP);
  IMU = wb_robot_get_device("IMU");
  wb_inertial_unit_enable(IMU,TIME_STEP);
wb_joystick_enable(TIME_STEP);

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

    /* Process sensor data here */
    for (int num = 0;num < 4;num++)
    {
      pos =wb_position_sensor_get_value(sensor[num]);
      // if (pos < -1.5)
      // {
        // flag1--;
      // }
      // if (pos > 1.5)
      // {
        // flag1++;
      // }
      // if (flag1 > 10)
      // {
      // flag1 = 3;
      // }
      // if (flag1 < -10)
      // {
      // flag1 = -3;
      // }
      // if (pos > 0)
      // {
        Chassis.Motor_Pos[num] = pos ;
        // //+ pow(1.57,flag1-1);
      // }
      // else 
      // {
        // Chassis.Motor_Pos[num] = pos ;
        // //- pow(1.57,flag1 +1);
      // }
      // printf("motor_pos=%.2f\n",-wb_position_sensor_get_value(sensor[num]));
      printf("Chassis.Motor_Pos[num]=%f\n",Chassis.Motor_Pos[num]);
    }
    angle = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    //printf("angle %.2f %.2f %.2f\n",angle[0],angle[1],angle[2]);

   Vel_X = (float)wb_joystick_get_axis_value(1) / 35536.0f * 20;
   Vel_Y = -(float)wb_joystick_get_axis_value(0) / 35536.0f * 20;
   Vel_W = (float)wb_joystick_get_axis_value(2) / 35536.0f * 20;
    if(Vel_X < 1 && Vel_X > -1) Vel_X = 0;
    if(Vel_Y < 1 && Vel_Y > -1) Vel_Y = 0;
    if(Vel_W < 1 && Vel_W > -1) Vel_W = 0;
    printf("%.2f %.2f %.2f\n",Vel_Y,Vel_X,Vel_W);
 
    if(mode == 0)
    {
   FourSteering_Chassis_IK_Cal(Vel_Y,Vel_X,Vel_W);

    }else if(mode == 1)
    {
    IK();
    FourSteering_Chassis_IK_Cal(Vel_Y,Vel_X,Vel_W);
    }
    Motor_Angle();
   Motor_Speed();
    wb_motor_set_velocity(motors[0],Chassis.Chassis_Control_Motor.Motor_Move[0].Tar_Vel );
    wb_motor_set_velocity(motors[1],Chassis.Chassis_Control_Motor.Motor_Move[1].Tar_Vel );
    wb_motor_set_velocity(motors[2],Chassis.Chassis_Control_Motor.Motor_Move[2].Tar_Vel );
    wb_motor_set_velocity(motors[3],Chassis.Chassis_Control_Motor.Motor_Move[3].Tar_Vel );
    wb_motor_set_position(motors[4],Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Pos);
    wb_motor_set_position(motors[5],Chassis.Chassis_Control_Motor.Motor_Turn[1].Tar_Pos);
    wb_motor_set_position(motors[6],Chassis.Chassis_Control_Motor.Motor_Turn[2].Tar_Pos);
    wb_motor_set_position(motors[7],Chassis.Chassis_Control_Motor.Motor_Turn[3].Tar_Pos);
    printf("%.2f %.2f %.2f\n",Vel_Y,Vel_X,Vel_W);
printf("OK1 Motor_Turn[0]=%.2f,Motor_Move[0]=%.2f\n",Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Pos,Chassis.Chassis_Control_Motor.Motor_Move[0].Tar_Vel);
    // wb_motor_set_position(motors[4],1);
    // wb_motor_set_position(motors[5],1);
    // wb_motor_set_position(motors[6],1);
    // wb_motor_set_position(motors[7],1);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

