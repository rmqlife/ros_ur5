#include "sunrise/sriCommManager.h"
#include <iostream>
#include <fstream>
#include<ctime>
// #include<windows.h>
using namespace std;
clock_t startTime, endTime;
int T = 1;
int temp_max_time = 0;
int all_time = 1;
float force_1 = 0;
float FZ =0;
float forcenumber[5000] = { NULL };
float maxnuumber[500] = { NULL };
CSRICommManager::CSRICommManager()
{
}


CSRICommManager::~CSRICommManager()
{
}

bool CSRICommManager::Init()
{
	mTCPClient.OpenTCP("192.168.0.108", 4008);
	mTCPClient.AddCommParser(&mATParser);
	mTCPClient.AddCommParser(&mM8218Parser);
	//bind Network communication failure processing function
	
	SRICommNetworkFailureCallbackFunction networkFailureCallback = std::bind(&CSRICommManager::OnNetworkFailure, this, std::placeholders::_1);
	mTCPClient.SetNetworkFailureCallbackFunction(networkFailureCallback);
	//bind ACK command Processing function
	
	SRICommATCallbackFunction atCallback = std::bind(&CSRICommManager::OnCommACK, this, std::placeholders::_1);
	mATParser.SetATCallbackFunction(atCallback);
	//bind M8128 Data processing function

	SRICommM8218CallbackFunction m8218Callback = std::bind(&CSRICommManager::OnCommM8218, this, 
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
		std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
	mM8218Parser.SetM8218CallbackFunction(m8218Callback);

	return true;
}

bool  CSRICommManager::Run()
{
// 	creat a TCP client connection

	if (mTCPClient.Connect() == false)
	{
	
		return false;

	}
	//Send command to M8128 to set data upload format
	
	if (SendCommand("SGDM", "(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)") == false)
	{
		return false;
	}
	//Send command to M8128 to set data check mode
	
	if (SendCommand("DCKMD", "SUM") == false)
	{
		return false;
	}
	
	if (SendCommand("SMPF", "500") == false)
	{
return false;
	}
	
	if (SendCommand("ADJZF", "1;1;1;1;1;1;") == false)
	{
		return false;
	};
		if (SendCommand("GSD", "") == true)
	                {
						
	                } 
	return true;
}

bool CSRICommManager::Stop()
{
	mTCPClient.CloseTCP();
	return true;
}


bool CSRICommManager::OnNetworkFailure(std::string infor)
{
	printf("OnNetworkFailure = %s\n", infor.data());
	return true;
}


//send command 

bool CSRICommManager::SendCommand(std::string command, std::string parames)
{
	mIsGetACK = false;
	mCommandACK = "";
	mParamesACK = "";

	//Combination command

	std::string atCommand = "AT+" + command + "=" + parames + "\r\n";
	if (mTCPClient.OnSendData((BYTE*)atCommand.data(), (int)atCommand.length()) == false)
	{
		return false;
	}
	//wait ACK
// #ifdef  IS_WINDOWS_OS
// 	std::clock_t start = clock();
// 	while (true)
// 	{
// 		if (mIsGetACK == true)
// 		{
// 			break;
// 		}
// 		std::clock_t end = clock();
// 		long span = end - start;
// 		if (span >= 1000)//10s
// 		{
// 			return false;
// 		}
// 	}
// #else

	timeval start, end;
	gettimeofday(&start, NULL);
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		gettimeofday(&end, NULL);
		long span = 1000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000;
		if (span >= 10000)//10s
		{
			return false;
		}
	}
// #endif
	//
	if (mCommandACK != command)
	{
		//ACK command error
		return false;
	}
	//
	printf("ACK+%s=%s", mCommandACK.data(), mParamesACK.data());
	//
	return true;
}

//ACK command processing
bool CSRICommManager::OnCommACK(std::string command)
{
	int index = (int)command.find('=');
	if (index == -1)
	{
		mCommandACK = command;
		mParamesACK = "";
	}
	else
	{
		mCommandACK = command.substr(0, index);
		mParamesACK = command.substr(index+1);
	}
	mCommandACK = mCommandACK.substr(4);	
	
	mIsGetACK = true;

	return true;
}
