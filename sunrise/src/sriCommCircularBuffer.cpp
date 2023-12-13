#include "sunrise/sriCommCircularBuffer.h"
// #include"sriCommDefine.h"
#include<algorithm>
// #include<math.h>
using namespace std;

CSRICommCircularBuffer::CSRICommCircularBuffer()
{
	mWIndex = 0;
	mRIndex = 0;
	mTotalWCount = 0;
	mTotalRCount = 0;
	mBufferSize = 0;
	mBuffer = NULL;
}


CSRICommCircularBuffer::~CSRICommCircularBuffer()
{
	if (mBuffer != NULL)
	{
		delete mBuffer;
	}
}
//Init the circular buffer

bool CSRICommCircularBuffer::Init(int bufferMaxSize)
{
	if (bufferMaxSize <= 0)
	{
		return false;
	}
	mWIndex = 0;
	mRIndex = 0;
	mTotalWCount = 0;
	mTotalRCount = 0;
	mBufferSize = bufferMaxSize;
	mBuffer = new BYTE[mBufferSize];
	return true;
}

//Get the circular buffer data length

int CSRICommCircularBuffer::GetLength()
{
	int len = 0;

	int wIndex = mWIndex;
	int rIndex = mRIndex;

	if (wIndex == rIndex)
	{
		len = 0;
	}
	else if (wIndex > rIndex)
	{
		len = wIndex - rIndex;
	}
	else
	{
		len = mBufferSize - rIndex + wIndex;
	}
	return len;
}
//Get the circular buffer data length

int CSRICommCircularBuffer::GetLength(int& wIndex, int& rIndex)
{
	int len = 0;
	//
	wIndex = mWIndex;
	rIndex = mRIndex;

	if (wIndex == rIndex)
	{
		len = 0;
	}
	else if (wIndex > rIndex)
	{
		len = wIndex - rIndex;
	}
	else
	{
		len = mBufferSize - rIndex + wIndex;
	}
	return len;
}
//Clear circular buffer

bool CSRICommCircularBuffer::Clear()
{
	mWIndex = 0;
	mRIndex = 0;

	return true;
}
//Clear data of specified length

bool CSRICommCircularBuffer::Clear(int clearLen)
{
	if (clearLen < 0)
	{
		return false;
	}
	if (clearLen == 0)
	{
		return true;
	}
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	if (clearLen > length)
	{
		return Clear();
	}
	else
	{
		int rIndexNew = (rIndex + clearLen) % mBufferSize;
		//
		mRIndex = rIndexNew;
		//
	}
	return true;
}
//Write to circular buffer

int CSRICommCircularBuffer::Write(BYTE* data, int dataLen)
{
	int len = dataLen;
	if (len <= 0)
	{
		return 0;
	}
	if (len > mBufferSize)
	{
		len = mBufferSize;
	}
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	int spaceLen = mBufferSize - length - 1;
	if (len > spaceLen)
	{
		return 0;
	}
	int wIndexNew = 0;
	//When the write data address exceeds the buffer size
	
	if (wIndex + len <= mBufferSize)
	{
		memcpy(mBuffer + wIndex, data, len);
		wIndexNew = (wIndex + len) % mBufferSize;
	}
	else
	{
		int firstBlockLen = mBufferSize - wIndex;
		int secondBlockLen = len - firstBlockLen;
		memcpy(mBuffer + wIndex, data, firstBlockLen);
		memcpy(mBuffer, data + firstBlockLen, secondBlockLen);
		wIndexNew = (wIndex + len) % mBufferSize;
	}
	mWIndex = wIndexNew;
	//
	mTotalWCount += len;
	return len;
}
//Write a single byte of data to the circular buffer

int CSRICommCircularBuffer::Write(BYTE data)
{
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	int spaceLen = mBufferSize - length - 1;
	if (1 > spaceLen)
	{
		return 0;
	}

	mBuffer[wIndex] = data;
	wIndex++;
	if (wIndex >= mBufferSize)
	{
		wIndex = 0;
	}
	mWIndex = wIndex;
	mTotalWCount += 1;
	return 1;
}

//Read the circular buffer data

BYTE* CSRICommCircularBuffer::Read(int& dataLen, int readLen, bool delData)
{
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	int len = 0;
	//When no read length is specified, all data is read by default
	
	if (readLen <= 0)
	{
		len = length;
	}
	else
	{
		len =std::min(length, readLen);
	}
	if (len <= 0)
	{
		return NULL;
	}
	BYTE* data = new BYTE[len];
	dataLen = len;
	//When the read data address exceeds the buffer size
	
	if (rIndex + len <= mBufferSize)
	{
		memcpy(data, mBuffer + rIndex, len);
		if (delData == true)
		{
			long rIndexNew = (rIndex + len) % mBufferSize;
			mRIndex = rIndexNew;
		}
	}
	else
	{
		long firstBlockLen = mBufferSize - rIndex;
		long secondBlockLen = len - firstBlockLen;
		memcpy(data, mBuffer + rIndex, firstBlockLen);
		memcpy(data + firstBlockLen, mBuffer, secondBlockLen);
		if (delData == true)
		{
			long rIndexNew = (rIndex + len) % mBufferSize;
			mRIndex = rIndexNew;
		}
	}
	mTotalRCount += len;
	return data;
}


//read circular buffer data

BYTE* CSRICommCircularBuffer::ReadTry(int& dataLen, int readLen)
{
	return Read(dataLen, readLen, false);
}