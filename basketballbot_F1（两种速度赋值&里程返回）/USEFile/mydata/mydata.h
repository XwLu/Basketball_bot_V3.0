 #ifndef _MYDATA_H_  
#define _MYDATA_H_  


typedef struct motor{
	float M1;
	float M2;
	float M3;
	float M4;
} motor;


typedef struct vector{
	float x;
	float y;
	float z;
}vector;


typedef struct robot{
	//速度
  motor single_real_speed;
  motor single_expected_speed;
  vector globel_real_speed;
	vector globel_expected_speed;
	vector robot_expected_speed;
	vector robot_real_speed;
	//状态
	enum Recstate		//控制方式
	{
		Global,Robot
	} Control_ID;
	
	//里程
	motor encoder_data;
  vector real_position;
	vector expected_position;
}robot;




#endif 


