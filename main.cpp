#include "controlPosition.h"
#include "processMavlink.h"
#include "controlSocket.h"
#include <iostream>
#include <tchar.h>
#include <conio.h>
using namespace std;


int _tmain(int argc, _TCHAR* argv[])
{

	CMavLinkProcessor myMavLinkProcessor1;	//Serial Port 1, PIXHAWK
	CMavLinkProcessor myMavLinkProcessor2;	//Serial Port 2, GCS
	
	Mode myMode;
	SocketMode mySocketMode;
	CPositionController myPositionController(&myMavLinkProcessor1, &myMavLinkProcessor2, &myMode);
	CSocketController mySocketController(&myMavLinkProcessor1, &myMavLinkProcessor2, &mySocketMode);

	if (!myMavLinkProcessor1.InitPort(COMNUM1,BAUD1))
		std::cout << "initPort fail !" << std::endl;
	else
		std::cout << "initPort succeed !" << std::endl;

	if (!myMavLinkProcessor1.OpenListenThread1())
		std::cout << "OpenListenThread fail !" << std::endl;
	else
		std::cout << "OpenListenThread succeed !" << std::endl;

	

	if (!myMavLinkProcessor2.InitPort(COMNUM2, BAUD2))
		std::cout << "initPort fail !" << std::endl;
	else
		std::cout << "initPort succeed !" << std::endl;

	if (!myMavLinkProcessor2.OpenListenThread2())
		std::cout << "OpenListenThread fail !" << std::endl;
	else
		std::cout << "OpenListenThread succeed !" << std::endl;



	//////////////////////
	if (!myPositionController.OpenControllerThread())
		cout << "open Controller thread fail !" << endl;
	else
		cout << "open Controller thread succeed !" << endl;

	if (!mySocketController.OpenSocketThread())
		cout << "open Socket thread fail !" << endl;
	else
		cout << "open Socket thread succeed !" << endl;



	while (1)
	{

		if (myMavLinkProcessor2.isGCStalk)
		{
			myMode = DELIVER_GCS_COMMAND_TO_HAWK;
		}
		else
		{
			myMode = CONTROL_NOTHING;
			//myMode = DELIVER_COPTER_INFO_TO_GCS;
		}


	}
	getchar();
	return 0;
}