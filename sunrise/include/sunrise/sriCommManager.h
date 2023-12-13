#ifndef SRI_COMM_MANAGER_H
#define SRI_COMM_MANAGER_H


#include "sriCommDefine.h"
#include "sriCommTCPClient.h"
#include "sriCommATParser.h"
#include "sriCommM8218Parser.h"
#include <iostream>
#include <fstream>

class CSRICommManager
{

public:
	CSRICommManager();
	~CSRICommManager();

	float output[10]={NULL};	
	bool Init();
	bool Stop();
	bool Run();
	int  Datainterception();
	bool SendCommand(std::string command, std::string parames);
	

	bool OnNetworkFailure(std::string infor);//ͨѶʧ��
	bool OnCommACK(std::string command);//ACKӦ�����ݴ���
	bool OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz)
	{
	 output[10]={NULL};	
			output[0]=fx;
			output[1]=fy;
			output[2]=fz;
			output[3]=mx;
			output[4]=my;
			output[5]=mz;
			return true;
	}//GSD���ݴ���

// float *sixnumber()
// {
// 	return output;
// }

private:
	CSRICommTCPClient mTCPClient;
	CSRICommATParser mATParser;//ATָ�������
	CSRICommM8218Parser mM8218Parser;//GSD���ݽ�����
	bool mIsGetACK;
	std::string mCommandACK;
	std::string mParamesACK;
	std::ofstream dataFile;
	
};

#endif

