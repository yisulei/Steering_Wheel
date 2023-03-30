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
#define len_car 0.95
#define wid_car 0.95
#define PI 3.13159
#define Half_pi (PI/2)
#define Two_pi (PI*2)
#define abs(x) ((x)>0 ? (x):(-(x)))
#define A_car 0.321

double V_car;
double R_car;
float a;
float b;
float angle_car;
float angle_F;
float angle_RB;
float angle_LB;

float speed_F;
float speed_RB;
float speed_LB;
double Vel = 1;
double Vel_Y = 0;
double Vel_X = 0;
double Vel_W = 0;
int mode = 0;
float pos[3];
int flag1 = 1;
int flag2 = 1;
int cir[3] ;
WbDeviceTag motors[8];
WbDeviceTag sensor[3];
WbDeviceTag compass;
WbDeviceTag IMU;

char motors_names[6][30] = {"F_move", "LB_move", "RB_move","F_turn", "LB_turn", "RB_turn"};
char sensor_names[3][30] = {"position_sensor_F","position_sensor_LB","position_sensor_RB"};
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
	Motor_3510_TypeDef Motor_Turn[3];
	Motor_3508_TypeDef Motor_Move[3];
}Chassis_Control;

typedef struct Chassis_TypeDef
{
	FourSteering_Move_TypeDef Chassis_Motor_Tar;
	double Motor_Pos[3];
	Chassis_Control Chassis_Control_Motor;
	
}Chassis_TypeDef;
Chassis_TypeDef Chassis;

// void IK()
// {
  // Vel_X = Vel*sin(angle[2]);
  // Vel_Y = Vel*cos(angle[2]);
// }

	
void RemoteController_Analysis(void);
void Motor_Drive(void);
void Motor_Info_Analysis(void);
void Init(void);
void FourSteering_Chassis_IK_Cal(float Vel_Y,float Vel_X,float Vel_W);
int main(int argc, char **argv) {
  wb_robot_init();
  Init();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    // angle = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    Motor_Info_Analysis();
    RemoteController_Analysis();
    FourSteering_Chassis_IK_Cal(Vel_Y,Vel_X,Vel_W);
    Motor_Drive();
  }wb_robot_cleanup();

  return 0;
}

void Init(void)
{
  for (int i = 0; i < 6; i++) motors[i] = wb_robot_get_device(motors_names[i]);
  for (int i = 0;i < 3;i++)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i],0);
    sensor[i] = wb_robot_get_device(sensor_names[i]);
    wb_position_sensor_enable(sensor[i],TIME_STEP);
  }
  // compass = wb_robot_get_device("compass");
  // wb_compass_enable(compass,TIME_STEP);
  // IMU = wb_robot_get_device("IMU");
  // wb_inertial_unit_enable(IMU,TIME_STEP);
  wb_joystick_enable(TIME_STEP);
}

void RemoteController_Analysis(void)
{
   Vel_X = (float)wb_joystick_get_axis_value(1) / 35536.0f * 30;
   Vel_Y = (float)wb_joystick_get_axis_value(0) / 35536.0f * 30;
   Vel_W =  (float)wb_joystick_get_axis_value(3) / 35536.0f * 50;
   if(Vel_X < 1 && Vel_X > -1) Vel_X = 0;
   if(Vel_Y < 1 && Vel_Y > -1) Vel_Y = 0;
   if(Vel_W < 1 && Vel_W > -1) Vel_W = 0;
   printf("Vel_Y=%.2f Vel_X=%.2f Vel_W=%.2f\n",Vel_Y,Vel_X,Vel_W);
}

void Motor_Info_Analysis(void)
{
    for (int num = 0;num < 3;num++)
    {
      pos[num] =wb_position_sensor_get_value(sensor[num]);
        cir[num] = (int)(pos[num]/Two_pi);
        Chassis.Motor_Pos[num] = pos[num] ;
      // printf("Chassis.Motor_Pos[num]=%f,cir[%d],%d\n",Chassis.Motor_Pos[num],num,cir[num]);
    }
}

void Motor_Drive(void)
{
    wb_motor_set_velocity(motors[0],Chassis.Chassis_Control_Motor.Motor_Move[0].Tar_Vel );
    wb_motor_set_velocity(motors[1],Chassis.Chassis_Control_Motor.Motor_Move[1].Tar_Vel );
    wb_motor_set_velocity(motors[2],Chassis.Chassis_Control_Motor.Motor_Move[2].Tar_Vel );
    wb_motor_set_position(motors[3],Chassis.Chassis_Control_Motor.Motor_Turn[0].Tar_Pos);
    wb_motor_set_position(motors[4],Chassis.Chassis_Control_Motor.Motor_Turn[1].Tar_Pos);
    wb_motor_set_position(motors[5],Chassis.Chassis_Control_Motor.Motor_Turn[2].Tar_Pos);
}

void Motor_Angle()
{

	for (int num = 0;num < 3;num++)
	{
	printf("Chassis.Motor_Pos[%d]=%f,cir[%d],%d\n",num,Chassis.Motor_Pos[num],num,cir[num]);
	// printf("&&&&&Chassis.Chassis_Motor_Tar.Motor_Tar[%d]=%.2f\n",num,Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]);		
		if (Chassis.Motor_Pos[num] >= 0)
		{
		   // printf("%.2f %.2f\n",Chassis.Motor_Pos[num],Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]);
			if (((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) <= Half_pi) && ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) >= -Half_pi) )
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+3];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos= Chassis.Motor_Pos[num] + Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num];
				Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] > PI*3/2)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] += 2*PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]);
            			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] < Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] += PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + ((Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]));
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] > Chassis.Motor_Pos[num])
			{
			printf("********************************************************************\n");
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + ((Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]));
			//printf("Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] = %d");
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
		}
		else if (Chassis.Motor_Pos[num] < 0)
		{
			if ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]<= Half_pi) && (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) >= -Half_pi)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] = Chassis.Chassis_Motor_Tar.Motor_Tar[num+3];
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]);
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else if (Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] < -PI*3/2)
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] -= 2*PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]);
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] < Chassis.Motor_Pos[num])
			{
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] += PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]);
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			}
			else if (Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] > Chassis.Motor_Pos[num])
			{
			printf("Chassis.Chassis_Motor_Tar.Motor_Tar[%d+3]=%f\n",num,Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]);
				Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] -= PI;
				Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos = Chassis.Motor_Pos[num] + ((Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] - Chassis.Motor_Pos[num]));
			Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				printf("##############################################\n");
			} 
		}
		printf("Chassis.Chassis_Control_Motor.Motor_Turn[%d].Tar_Pos=%.2f,Chassis.Chassis_Control_Motor.Motor_Move[%d].Tar_Vel=%.2f\n",num,Chassis.Chassis_Control_Motor.Motor_Turn[num].Tar_Pos,num,Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel);		
	}
}

void Motor_Speed()
{
	// for (int num = 0;num < 3;num++)
	// {
		// if (Chassis.Motor_Pos[num] > 0)
		// {
			// if (((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] <= Half_pi) && ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) > -Half_pi ))|| Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] > PI*3/2)
			// {
				// Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				// Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			// }
			// else 
			// {
			// printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				// Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				// Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			// }
		// }
		// else if (Chassis.Motor_Pos[num] < 0)
		// {
			// if (((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] < Half_pi) && ((Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3]) > -Half_pi)) || Chassis.Motor_Pos[num] - Chassis.Chassis_Motor_Tar.Motor_Tar[num+3] < -PI*3/2)
			// {
				// Chassis.Chassis_Motor_Tar.Motor_Tar[num] = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				// Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			// }
			// else 
			// {
			// printf("1123/n");
				// Chassis.Chassis_Motor_Tar.Motor_Tar[num] = -Chassis.Chassis_Motor_Tar.Motor_Tar[num];
				// Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel = Chassis.Chassis_Motor_Tar.Motor_Tar[num];
			// }
		// }
		// printf("Motor_speed=%.2f \n",Chassis.Chassis_Control_Motor.Motor_Move[num].Tar_Vel);
    // }
}

void FourSteering_Chassis_IK_Cal(float Vel_Y,float Vel_X,float Vel_W)
{
	V_car = sqrt(Vel_X*Vel_X + Vel_Y*Vel_Y);
	angle_car = atan2(Vel_X , Vel_Y) ;
	if(angle_car >0)
	{
	angle_car =PI-angle_car;
	}
	else if(angle_car <0)
	{
	angle_car = -PI - angle_car;
	}
	printf("angle_car=%f\n",angle_car);
            if(Vel_W == 0 && (Vel_Y != 0 || Vel_X != 0))
	{
              angle_F = -angle_car;
              angle_LB = -angle_car;
              angle_RB = -angle_car;
              speed_F = V_car;
              speed_LB = V_car;
              speed_RB =-V_car;
	}
	else if(Vel_W != 0 && (Vel_Y == 0 && Vel_X == 0))
	{
              angle_F = Half_pi;
              angle_LB = PI*1/6;
              angle_RB = -PI*1/6;
              speed_F = Vel_W*A_car;
              speed_LB = Vel_W*A_car;
              speed_RB = Vel_W*A_car;
	}
	else if(Vel_W != 0 && (Vel_Y != 0 || Vel_X != 0))
	{
              R_car = abs(V_car / Vel_W);
              b = R_car * sin(angle_car);
              a = R_car * cos(angle_car);
              printf("V_car=%.2f Vel_W=%.2f %.2f\n",V_car,Vel_W,cos(angle_car));
              if(Vel_W<0)
              {
                angle_F = -atan2(-(A_car - b), a );
                angle_LB = -atan2((b + A_car / 2) , (a - A_car * 1.7f / 2)  );
                angle_RB = -atan2((b + A_car / 2) , (a + A_car * 1.7f / 2)  );
                // angle_RB = -atan2((b + A_car / 2) , (a - A_car * 1.7f / 2)  );
                // angle_LB = -atan2((b + A_car / 2) , (a + A_car * 1.7f / 2)  );
                speed_F = sqrt((b - A_car) * (b - A_car) + a * a) / R_car *abs(V_car);
                speed_LB = sqrt((b + A_car / 2) * (b + A_car / 2) + (a - A_car * 1.7f / 2) * (a - A_car * 1.7f / 2)) / R_car * abs(V_car);
                speed_RB = -sqrt((b + A_car / 2) * (b + A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * abs(V_car);
              }
              else
              {
                 angle_F = -atan2(b + A_car, a );
                angle_LB = -atan2((b - A_car / 2) , (a + A_car * 1.7f / 2)  );
                angle_RB = -atan2((b - A_car / 2) , (a - A_car * 1.7f / 2)  );
                speed_F = sqrt((b + A_car) * (b + A_car) + a * a) / R_car * abs(V_car);
                speed_LB = sqrt((b - A_car / 2) * (b - A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)) / R_car * abs(V_car);
                speed_RB = -sqrt((b - A_car / 2) * (b - A_car / 2) + (a - A_car * 1.7f / 2) * (a - A_car * 1.7f / 2)) / R_car * abs(V_car);
                // printf("angle_F=%f,angle_LB=%f,angle_RB=%f",angle_F,angle_LB,angle_RB);
              }
	}
	else
	{
              speed_F = 0;
              speed_LB = 0;
              speed_RB = 0;
	}
	Chassis.Chassis_Motor_Tar.Motor_Tar[0] = speed_F;
	Chassis.Chassis_Motor_Tar.Motor_Tar[1] = speed_LB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[2] = speed_RB;
	Chassis.Chassis_Motor_Tar.Motor_Tar[3] = angle_F +cir[0]*Two_pi;
	Chassis.Chassis_Motor_Tar.Motor_Tar[4] = angle_LB +cir[1]*Two_pi;
	Chassis.Chassis_Motor_Tar.Motor_Tar[5] = angle_RB +cir[2]*Two_pi;
	printf("angle_F=%f,angle_LB=%f,angle_RB=%f,R_car=%f,V_car=%f,Vel_W=%f,x_F=%f,x_LB=%f,x_RB=%f,speed_F=%f,speed_LB=%f,speed_RB=%f\n",angle_F,angle_LB,angle_RB,R_car,V_car,Vel_W,sqrt((b + A_car) * (b + A_car) + a * a),sqrt((b - A_car / 2) * (b - A_car / 2) + (a + A_car * 1.7f / 2) * (a + A_car * 1.7f / 2)),sqrt((b - A_car / 2) * (b - A_car / 2) + (a - A_car * 1.7f / 2) * (a - A_car * 1.7f / 2)),speed_F,speed_LB,speed_RB);
	
	Motor_Angle();
           Motor_Speed();
}