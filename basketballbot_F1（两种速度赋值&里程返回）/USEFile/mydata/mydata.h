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
	//�ٶ�
  motor single_real_speed;
  motor single_expected_speed;
  vector globel_real_speed;
	vector globel_expected_speed;
	vector robot_expected_speed;
	vector robot_real_speed;
	//״̬
	enum Recstate		//���Ʒ�ʽ
	{
		Global,Robot
	} Control_ID;
	
	//���
	motor encoder_data;
  vector real_position;
	vector expected_position;
}robot;




#endif 


