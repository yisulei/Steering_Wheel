#include "stm32f4xx.h"
#include "stdio.h"
#include "ifr_pid.c"
#include "math.h"
int OUTPUT_W;
int OUTPUT_Y;

typedef struct TF40		//����λ�û����Ƶĵ����Ϣ
{
	uint16_t distance_L;
	uint16_t distance_R;
}TF40;
int target_pos[5];
typedef struct TF40_struct		//����λ�û����Ƶĵ����Ϣ
{
	TF40 tf40;
	float Tar_theta;
	PID_TypeDef W_PID;					//�ٶȻ�PID����
	PID_TypeDef Y_PID;					//�ٶȻ�PID����
	float Speed_W;
	float Speed_Y;
	float difference_theta;
}TF40_struct;



TF40_struct Chassis_TF40;
void TF40_PID_Init()       
{
	IFR_PID_Param_Init(&Chassis_TF40.W_PID ,	20,	0,	15,	90,	3,	0,	1000);
	IFR_PID_Param_Init(&Chassis_TF40.Y_PID ,	2,	0.05,	10,	1000,	0,	1000,	1500);	
}
int distance_Tf40=0;
void Chassis_TF40_Aim()
{
	int distance_r=0,distance_l=0,interval_TF40=410;//mm
    distance_r=Chassis_TF40.tf40.distance_R;
	distance_l=Chassis_TF40.tf40.distance_L;
    distance_Tf40=(distance_l+distance_r)/2;
	Chassis_TF40.difference_theta =atan2((distance_l-distance_r),interval_TF40)*57.325f;
	Chassis_TF40.Tar_theta=0;
   	IFR_Incremental_PID(&Chassis_TF40.W_PID,Chassis_TF40.Tar_theta,Chassis_TF40.difference_theta);
    OUTPUT_W=Chassis_TF40.W_PID.Output;   //�˴�ע��xy�ٶ�ӦΪ0 
	IFR_Incremental_PID(&Chassis_TF40.W_PID, distance_Tf40. , target_pos[0]);//Ŀ��λ������� 
	OUTPUT_Y=Chassis_TF40.Y_PID.Output;   //�˴�ע��xw�ٶ�ӦΪ0 
}



