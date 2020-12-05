/****************************************************
	Virtual Collaborative Teams - The base program 
    The main module prepared for state prediction topic
****************************************************/

#include <string>
bool if_prediction_test = true;
bool if_delays = false;
bool if_shadow = true;
// test prediction scenario - e.g. benchmark to compare different prediction algorithms
// {time [s], force [N], wheel turning speed [rad/s], breaking degree} 
float test_scenario[][4] = { { 9.5, 100, 0, 0 }, { 5, 20, -0.25 / 8, 0 }, { 0.5, 0, 0, 1.0 }, { 5, 60, 0.25 / 8, 0 }, { 15, 100, 0, 0 } };
//float test_scenario[][4] = { { 9.5, 500, 0, 0 }, { 10, -200, -0.25 / 2, 0 } };  // more extremal version


#include <windows.h>
#include <math.h>
#include <time.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include <iterator> 
#include <map>

#include "objects.h"
#include "graphics.h"
#include "net.h"
using namespace std;

FILE *f = fopen("vct__log.txt", "a");      // log file handle


MovableObject *my_vehicle;                 // vehicle controlled in this application
Terrain terrain;						   // environment: terrain, things (trees, coins etc.)


map<int, MovableObject*> movable_objects;  // objects from other applications indexed by iID

float fDt;                                 // average time between two consecutive cycles of world (simulation + rendering)
										   // is used to synchronize simulated phisical time in each application despite of
										   // hardware or processes number differences between computers 
long time_of_cycle, number_of_cyc=0;       // variables supported fDt calculation
long time_start = clock();           
long time_last_send = 0;
long time_last_received = 0;

multicast_net *multi_reciv;          // object (see net module) to recive messages from other applications
multicast_net *multi_send;           // ...  to send messages ...

HANDLE threadReciv;                  
HWND window_handle;                    
CRITICAL_SECTION m_cs;               

bool if_SHIFT_pressed = false;
bool if_ID_visible = true;           
bool if_mouse_control = false;      
int mouse_cursor_x = 0, mouse_cursor_y = 0;     

extern ViewParams view_parameters;           

long time_of_day = 1100;                    // day duration time

// variables due to state prediction
long number_of_send_trials = 0;        
float sum_differences_of_pos = 0;                 // the sum of distances between vehicle and its network shadow
float sum_of_angle_differences = 0;               // the sum of angular distances ...


struct Frame                                      // The main structure of net communication between aplications.
{	
	int iID;                                      // object identifier 
	int type;                                     // frame type  
	ObjectState state;                            // object state values (see object module)

	long sending_time;                            // timestamp - the send time
};


//******************************************
// The function of handling the message receiving thread
DWORD WINAPI ReceiveThreadFun(void *ptr)
{
	
	multicast_net *pmt_net = (multicast_net*)ptr;  // the pointer to the object of multicast_net class (see net module)
	Frame frame;

	while (1)
	{
		int frame_size = pmt_net->reciv((char*)&frame, sizeof(Frame));

		// waiting for frame 
		ObjectState state = frame.state;

		//fprintf(f, "odebrano stan iID = %d, ID dla mojego obiektu = %d\n", frame.iID, my_vehicle->iID);

		// Lock the Critical section
		EnterCriticalSection(&m_cs);               
	                                              

		if ((if_shadow) || (frame.iID != my_vehicle->iID))         
		{
			time_last_received = clock();
			if ((movable_objects.size() == 0) || (movable_objects[frame.iID] == NULL))       
				
			{
				MovableObject *ob = new MovableObject();
				ob->iID = frame.iID;
				movable_objects[frame.iID] = ob;		// registration of new object 
				//fprintf(f, "alien object ID = %d was registred\n", ob->iID);
			}
			movable_objects[frame.iID]->ChangeState(state);   // aktualizacja stateu obiektu obcego 	
			
		}	
		//Release the Critical section
		LeaveCriticalSection(&m_cs);               
	}  // while(1)
	return 1;
}

// *****************************************************************
// ****   All events which should be initialized once at the start of application    
void InteractionInitialisation()
{
	DWORD dwThreadId;

	my_vehicle = new MovableObject();    // creating my own object 

	time_of_cycle = clock();             // current time


	multi_reciv = new multicast_net("224.12.13.174", 10001);     
	multi_send = new multicast_net("224.12.13.174", 10001);      


	// creating a thread to handling messages from other aplications
	threadReciv = CreateThread(
		NULL,                        // no security attributes
		0,                           // use default stack size
		ReceiveThreadFun,                // thread function
		(void *)multi_reciv,               // argument to thread function
		NULL,                        // use default creation flags
		&dwThreadId);                // returns the thread identifier
	SetThreadPriority(threadReciv, THREAD_PRIORITY_HIGHEST);



	fprintf(f,"poczatek interakcji\n");
}


// *****************************************************************
// ****     All things (without graphics) to do as frequent as possible due to world cycle
void VirtualWorldCycle()
{
	number_of_cyc++;
	float time_from_start_in_s = (float)(clock() - time_start) / CLOCKS_PER_SEC;  // time from start of application

	if (number_of_cyc % 50 == 0)          
	{                              
		char text[256];
		long prev_time = time_of_cycle;
		time_of_cycle = clock();
		float fFps = (50 * CLOCKS_PER_SEC) / (float)(time_of_cycle - prev_time);
		if (fFps != 0) fDt = 1.0 / fFps; else fDt = 1;
	
		sprintf(text, "VCT 2020/21 ver. e, avg.freq = %0.2f[frame/s]  avg.dist = %0.3f[m]  avg.ang.dist = %0.3f[deg]",
			(float)number_of_send_trials / time_from_start_in_s, sum_differences_of_pos / number_of_cyc, 
			sum_of_angle_differences / number_of_cyc*180.0 / 3.14159);
		
		if (time_from_start_in_s > 5)
			SetWindowText(window_handle, text); 				
	}

	// distances calculation:
	EnterCriticalSection(&m_cs);
	MovableObject *car = (movable_objects.size() > 0 ? movable_objects[my_vehicle->iID] : NULL);       
	if (car != NULL)
	{
		sum_differences_of_pos += DistanceBetweenPointsOnTetraMap(my_vehicle->state.vPos, car->state.vPos);
		sum_of_angle_differences += AngleBetweenQuats(my_vehicle->state.qOrient, car->state.qOrient);
	}
	else {
		sum_differences_of_pos += DistanceBetweenPointsOnTetraMap(my_vehicle->state.vPos, Vector3(0,0,0));  
		sum_of_angle_differences += AngleBetweenQuats(my_vehicle->state.qOrient, quaternion(0, 0, 0, 1));
	}
	LeaveCriticalSection(&m_cs);
	
	// prediction test
	if (if_prediction_test)
	{
		int number_of_actions = sizeof(test_scenario) / (4 * sizeof(float));
		bool test_finished = test_scenario_step(my_vehicle, test_scenario, number_of_actions, time_from_start_in_s);

		if (test_finished) 
		{
			if_prediction_test = false;
			char text[200];
			sprintf(text, "After time %3.2f[s]  avg.freq = %0.2f[frame/s]  avg.dist = %0.3f[m]  avg.ang.dist = %0.3f[deg]",
				time_from_start_in_s, (float)number_of_send_trials / time_from_start_in_s, 
				sum_differences_of_pos / number_of_cyc, sum_of_angle_differences / number_of_cyc*180.0 / 3.14159);
			fprintf(f, "%s\n", text);
			MessageBox(window_handle, text, "Prediction test", MB_OK);
		}
	}

	my_vehicle->Simulation(fDt);                    

	time_from_start_in_s = (float)(clock() - time_start) / CLOCKS_PER_SEC;
	
	if ((float)(clock() - time_last_send) / CLOCKS_PER_SEC >= 1.0)
	{
		Frame frame;
		frame.state = my_vehicle->State();                 
		frame.iID = my_vehicle->iID;
		multi_send->send((char*)&frame, sizeof(Frame));  
		time_last_send = clock();
		number_of_send_trials++;
	}

	//             ----------------------------
	//          -------------------------------
	//       ----------------------------------
	//    -------------------------------------
	// ----------------------------------------
	// ------------  The place for state prediction:
	// Lock the Critical section
	EnterCriticalSection(&m_cs);
	for (map<int, MovableObject*>::iterator it = movable_objects.begin(); it != movable_objects.end(); ++it)
	{
		MovableObject *veh = it->second;
		if (veh)
		{
			quaternion qObrot = AsixToQuat(veh->state.vA_ang, veh->state.vA_ang.length());
			float deltaTime = (float)((clock() - time_last_received)/CLOCKS_PER_SEC);
			veh->state.vPos = veh->state.vPos + veh->state.vV * deltaTime;
			//veh->state.vV = my_vehicle->state.vV;
			//veh->state.vV = my_vehicle->state.vV;
			veh->state.qOrient = qObrot*veh->state.qOrient;
			//veh->state.vV_ang = my_vehicle->state.vV_ang;
		}
	}
	//Release the Critical section
	LeaveCriticalSection(&m_cs);
}


void EndOfInteraction()
{
	fprintf(f, "End of interaction.\n");
	fclose(f);
}


LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

HDC g_context = NULL;    



int WINAPI WinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPSTR     lpCmdLine,
	int       nCmdShow)
{
	
	//Initilize the critical section
	InitializeCriticalSection(&m_cs);

	MSG message;		
	WNDCLASS main_class; 

	static char class_name[] = "Main Class";

	main_class.style = CS_HREDRAW | CS_VREDRAW;
	main_class.lpfnWndProc = WndProc; 
	main_class.cbClsExtra = 0;
	main_class.cbWndExtra = 0;
	main_class.hInstance = hInstance; 
	main_class.hIcon = 0;
	main_class.hCursor = LoadCursor(0, IDC_ARROW);
	main_class.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);
	main_class.lpszMenuName = "Menu";
	main_class.lpszClassName = class_name;

	
	RegisterClass(&main_class);

	window_handle = CreateWindow(class_name, "VCT-lab 2020/21 ver. e - state prediction", WS_OVERLAPPEDWINDOW | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,
		500, 50, 750, 950, NULL, NULL, hInstance, NULL);

	ShowWindow(window_handle, nCmdShow);


	UpdateWindow(window_handle);


	ZeroMemory(&message, sizeof(message));
	while (message.message != WM_QUIT)
	{
		if (PeekMessage(&message, NULL, 0U, 0U, PM_REMOVE))
		{
			TranslateMessage(&message);
			DispatchMessage(&message);
		}
		else
		{
			VirtualWorldCycle();    // Cykl wirtualnego œwiata
			InvalidateRect(window_handle, NULL, FALSE);
		}
	}

	return (int)message.wParam;
}

/********************************************************************
FUNKCJA OKNA realizujaca przetwarzanie meldunków kierowanych do okna aplikacji*/
LRESULT CALLBACK WndProc(HWND window_handle, UINT message_code, WPARAM wParam, LPARAM lParam)
{

	switch (message_code)
	{
	case WM_CREATE:  //message wysy³any w momencie tworzenia okna
	{

		g_context = GetDC(window_handle);

		srand((unsigned)time(NULL));
		int result = GraphicsInitialisation(g_context);
		if (result == 0)
		{
			printf("nie udalo sie otworzyc okna graficznego\n");
			//exit(1);
		}

		InteractionInitialisation();

		SetTimer(window_handle, 1, 10, NULL);

		time_start = clock();      // by czas liczyæ po utworzeniu okna i inicjalizacji 

		return 0;
	}


	case WM_PAINT:
	{
		PAINTSTRUCT paint;
		HDC context;
		context = BeginPaint(window_handle, &paint);

		DrawScene();
		SwapBuffers(context);

		EndPaint(window_handle, &paint);

		return 0;
	}

	case WM_TIMER:

		return 0;

	case WM_SIZE:
	{
		int cx = LOWORD(lParam);
		int cy = HIWORD(lParam);

		WindowResize(cx, cy);

		return 0;
	}

	case WM_DESTROY: //obowi¹zkowa obs³uga meldunku o zamkniêciu okna

		EndOfInteraction();
		EndOfGraphics();

		ReleaseDC(window_handle, g_context);
		KillTimer(window_handle, 1);

		//LPDWORD lpExitCode;
		DWORD ExitCode;
		GetExitCodeThread(threadReciv, &ExitCode);
		TerminateThread(threadReciv,ExitCode);
		//ExitThread(ExitCode);

		//Sleep(1000);

		movable_objects.clear();
		

		PostQuitMessage(0);
		return 0;

	case WM_LBUTTONDOWN: //reakcja na lewy przycisk myszki
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		if (if_mouse_control)
			my_vehicle->F = 30.0;        // si³a pchaj¹ca do przodu
		break;
	}
	case WM_RBUTTONDOWN: //reakcja na prawy przycisk myszki
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		if (if_mouse_control)
			my_vehicle->F = -5.0;        // si³a pchaj¹ca do tylu
		break;
	}
	case WM_MBUTTONDOWN: //reakcja na œrodkowy przycisk myszki : uaktywnienie/dezaktywacja sterwania myszkowego
	{
		if_mouse_control = 1 - if_mouse_control;
		if (if_mouse_control) my_vehicle->if_keep_steer_wheel = true;
		else my_vehicle->if_keep_steer_wheel = false;

		mouse_cursor_x = LOWORD(lParam);
		mouse_cursor_y = HIWORD(lParam);
		break;
	}
	case WM_LBUTTONUP: //reakcja na puszczenie lewego przycisku myszki
	{
		if (if_mouse_control)
			my_vehicle->F = 0.0;        // si³a pchaj¹ca do przodu
		break;
	}
	case WM_RBUTTONUP: //reakcja na puszczenie lewy przycisk myszki
	{
		if (if_mouse_control)
			my_vehicle->F = 0.0;        // si³a pchaj¹ca do przodu
		break;
	}
	case WM_MOUSEMOVE:
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		if (if_mouse_control)
		{
			float wheel_angle = (float)(mouse_cursor_x - x) / 200;
			if (wheel_angle > my_vehicle->wheel_angle_max) wheel_angle = my_vehicle->wheel_angle_max;
			if (wheel_angle < -my_vehicle->wheel_angle_max) wheel_angle = -my_vehicle->wheel_angle_max;
			my_vehicle->state.wheel_angle = wheel_angle;
			//my_vehicle->steer_wheel_speed = (float)(mouse_cursor_x - x) / 20;
		}
		break;
	}
	case WM_KEYDOWN:
	{

		switch (LOWORD(wParam))
		{
		case VK_SHIFT:
		{
			if_SHIFT_pressed = 1;
			break;
		}
		case VK_SPACE:
		{
			my_vehicle->breaking_factor = 1.0;       // stopieñ hamowania (reszta zale¿y od si³y docisku i wsp. tarcia)
			break;                       // 1.0 to maksymalny stopieñ (np. zablokowanie kó³)
		}
		case VK_UP:
		{
			my_vehicle->F = 100.0;        // si³a pchaj¹ca do przodu
			break;
		}
		case VK_DOWN:
		{
			my_vehicle->F = -70.0;
			break;
		}
		case VK_LEFT:
		{
			if (my_vehicle->steer_wheel_speed < 0){
				my_vehicle->steer_wheel_speed = 0;
				my_vehicle->if_keep_steer_wheel = true;
			}
			else{
				if (if_SHIFT_pressed) my_vehicle->steer_wheel_speed = 0.5;
				else my_vehicle->steer_wheel_speed = 0.25 / 8;
			}

			break;
		}
		case VK_RIGHT:
		{
			if (my_vehicle->steer_wheel_speed > 0){
				my_vehicle->steer_wheel_speed = 0;
				my_vehicle->if_keep_steer_wheel = true;
			}
			else{
				if (if_SHIFT_pressed) my_vehicle->steer_wheel_speed = -0.5;
				else my_vehicle->steer_wheel_speed = -0.25 / 8;
			}
			break;
		}
		case 'I':   // wypisywanie nr ID
		{
			if_ID_visible = 1 - if_ID_visible;
			break;
		}
		case 'W':   // cam_distance widoku
		{
			//cam_pos = cam_pos - cam_direct*0.3;
			if (view_parameters.cam_distance > 0.5) view_parameters.cam_distance /= 1.2;
			else view_parameters.cam_distance = 0;
			break;
		}
		case 'S':   // przybli¿enie widoku
		{
			//cam_pos = cam_pos + cam_direct*0.3; 
			if (view_parameters.cam_distance > 0) view_parameters.cam_distance *= 1.2;
			else view_parameters.cam_distance = 0.5;
			break;
		}
		case 'Q':   // widok z góry
		{
			if (view_parameters.tracking) break;
			view_parameters.top_view = 1 - view_parameters.top_view;
			if (view_parameters.top_view)
			{
				view_parameters.cam_pos_1 = view_parameters.cam_pos; view_parameters.cam_direct_1 = view_parameters.cam_direct; view_parameters.cam_vertical_1 = view_parameters.cam_vertical;
				view_parameters.cam_distance_1 = view_parameters.cam_distance; view_parameters.cam_angle_1 = view_parameters.cam_angle;
				view_parameters.cam_pos = view_parameters.cam_pos_2; view_parameters.cam_direct = view_parameters.cam_direct_2; view_parameters.cam_vertical = view_parameters.cam_vertical_2;
				view_parameters.cam_distance = view_parameters.cam_distance_2; view_parameters.cam_angle = view_parameters.cam_angle_2;
			}
			else
			{
				view_parameters.cam_pos_2 = view_parameters.cam_pos; view_parameters.cam_direct_2 = view_parameters.cam_direct; view_parameters.cam_vertical_2 = view_parameters.cam_vertical;
				view_parameters.cam_distance_2 = view_parameters.cam_distance; view_parameters.cam_angle_2 = view_parameters.cam_angle;
				view_parameters.cam_pos = view_parameters.cam_pos_1; view_parameters.cam_direct = view_parameters.cam_direct_1; view_parameters.cam_vertical = view_parameters.cam_vertical_1;
				view_parameters.cam_distance = view_parameters.cam_distance_1; view_parameters.cam_angle = view_parameters.cam_angle_1;
			}
			break;
		}
		case 'E':   // obrót kamery ku górze (wzglêdem lokalnej osi z)
		{
			view_parameters.cam_angle += PI * 5 / 180;
			break;
		}
		case 'D':   // obrót kamery ku do³owi (wzglêdem lokalnej osi z)
		{
			view_parameters.cam_angle -= PI * 5 / 180;
			break;
		}
		case 'A':   // w³¹czanie, wy³¹czanie trybu œledzenia obiektu
		{
			view_parameters.tracking = 1 - view_parameters.tracking;
			if (view_parameters.tracking)
			{
				view_parameters.cam_distance = view_parameters.cam_distance_3; view_parameters.cam_angle = view_parameters.cam_angle_3;
			}
			else
			{
				view_parameters.cam_distance_3 = view_parameters.cam_distance; view_parameters.cam_angle_3 = view_parameters.cam_angle;
				view_parameters.top_view = 0;
				view_parameters.cam_pos = view_parameters.cam_pos_1; view_parameters.cam_direct = view_parameters.cam_direct_1; view_parameters.cam_vertical = view_parameters.cam_vertical_1;
				view_parameters.cam_distance = view_parameters.cam_distance_1; view_parameters.cam_angle = view_parameters.cam_angle_1;
			}
			break;
		}
		case 'Z':   // zoom - zmniejszenie k¹ta widzenia
		{
			view_parameters.zoom /= 1.1;
			RECT rc;
			GetClientRect(window_handle, &rc);
			WindowResize(rc.right - rc.left, rc.bottom - rc.top);
			break;
		}
		case 'X':   // zoom - zwiêkszenie k¹ta widzenia
		{
			view_parameters.zoom *= 1.1;
			RECT rc;
			GetClientRect(window_handle, &rc);
			WindowResize(rc.right - rc.left, rc.bottom - rc.top);
			break;
		}
		case VK_F1:  // wywolanie systemu pomocy
		{
			char lan[1024], lan_bie[1024];
			//GetSystemDirectory(lan_sys,1024);
			GetCurrentDirectory(1024, lan_bie);
			strcpy(lan, "C:\\Program Files\\Internet Explorer\\iexplore ");
			strcat(lan, lan_bie);
			strcat(lan, "\\pomoc.htm");
			int wyni = WinExec(lan, SW_NORMAL);
			if (wyni < 32)  // proba uruchominia pomocy nie powiodla sie
			{
				strcpy(lan, "C:\\Program Files\\Mozilla Firefox\\firefox ");
				strcat(lan, lan_bie);
				strcat(lan, "\\pomoc.htm");
				wyni = WinExec(lan, SW_NORMAL);
				if (wyni < 32)
				{
					char lan_win[1024];
					GetWindowsDirectory(lan_win, 1024);
					strcat(lan_win, "\\notepad pomoc.txt ");
					wyni = WinExec(lan_win, SW_NORMAL);
				}
			}
			break;
		}
		case VK_ESCAPE:
		{
			SendMessage(window_handle, WM_DESTROY, 0, 0);
			break;
		}
		} // switch po klawiszach

		break;
	}
	case WM_KEYUP:
	{
		switch (LOWORD(wParam))
		{
		case VK_SHIFT:
		{
			if_SHIFT_pressed = 0;
			break;
		}
		case VK_SPACE:
		{
			my_vehicle->breaking_factor = 0.0;
			break;
		}
		case VK_UP:
		{
			my_vehicle->F = 0.0;
			break;
		}
		case VK_DOWN:
		{
			my_vehicle->F = 0.0;
			break;
		}
		case VK_LEFT:
		{
			my_vehicle->Fb = 0.00;
			//my_vehicle->state.wheel_angle = 0;
			if (my_vehicle->if_keep_steer_wheel) my_vehicle->steer_wheel_speed = -0.25/8;
			else my_vehicle->steer_wheel_speed = 0; 
			my_vehicle->if_keep_steer_wheel = false;
			break;
		}
		case VK_RIGHT:
		{
			my_vehicle->Fb = 0.00;
			//my_vehicle->state.wheel_angle = 0;
			if (my_vehicle->if_keep_steer_wheel) my_vehicle->steer_wheel_speed = 0.25 / 8;
			else my_vehicle->steer_wheel_speed = 0;
			my_vehicle->if_keep_steer_wheel = false;
			break;
		}

		}

		break;
	}

	default: //statedardowa obs³uga pozosta³ych meldunków
		return DefWindowProc(window_handle, message_code, wParam, lParam);
	}


}

