// TestLine.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "TestLine.h"
#include <cmath>
#include <numbers>
#include <vector>
#include <windows.h>
#include <objidl.h>
#include <gdiplus.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

using namespace Gdiplus;
using namespace std;
#pragma comment (lib,"Gdiplus.lib")

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);


float MapRatio;
Point MapLeftCorner;
float OuterCityDiameter = 30;
float MapSize = OuterCityDiameter + 2;
float InnerCityDiameter = 6;
float RadialLineStep = 45.0;
float OuterCityStopLength = 4;
float InnerCityStopLength = OuterCityStopLength/4.0;
int lineWidth = 20;

typedef pair<float, float> MapPoint;

vector<vector<MapPoint>> SubwayMap;

class SubwayLane
{
public:

    Color color;
    string name;
    
public:

    SubwayLane(Color col, string nm)
    {
        color = col;
        name = nm;
    }

    SubwayLane(const SubwayLane& sl)
    {
        color = sl.color;
        name = sl.name;
    }
};

vector<SubwayLane> subwayLanes = 
{
    SubwayLane(Color(255, 255, 0, 0), "Bright Red"),
    SubwayLane(Color(255, 255, 165, 0), "Bright Orange"),
    SubwayLane(Color(255, 255, 255, 0), "Bright Yellow"),
    SubwayLane(Color(255, 0, 255, 0), "Bright Green"),
    SubwayLane(Color(255, 0, 255, 255), "Bright Blue"),
    SubwayLane(Color(255, 255, 0, 255), "Bright Magenta"),
    SubwayLane(Color(255, 0, 0, 0), "Black"),
    SubwayLane(Color(255, 255, 128, 0), "Bright Orange-Red"),
    SubwayLane(Color(255, 255, 255, 128), "Bright Yellow-Light"),
    SubwayLane(Color(255, 128, 255, 128), "Bright Lime Green"),
    SubwayLane(Color(255, 128, 128, 255), "Bright Periwinkle"),
    SubwayLane(Color(255, 255, 128, 255), "Bright Orchid"),
    SubwayLane(Color(255, 128, 255, 255), "Bright Turquoise"),
    SubwayLane(Color(255, 255, 0, 128), "Bright Hot Pink"),
    SubwayLane(Color(255, 128, 0, 255), "Bright Indigo"),
    SubwayLane(Color(255, 0, 0, 255), "Bright Light Blue") 
};

void GetMapSize(HWND hWnd)
{
    RECT windowSize;
    GetClientRect(hWnd, &windowSize);
    float screenSize = min(windowSize.bottom - windowSize.top, windowSize.right - windowSize.left);
    MapRatio = screenSize / MapSize;
    MapLeftCorner = Point(0, 0);
    int diff = abs(((windowSize.bottom - windowSize.top) - (windowSize.right - windowSize.left)) / 2);
    if (windowSize.bottom - windowSize.top > windowSize.right - windowSize.left) //taller than it is wide
        MapLeftCorner.Y = diff;
    else //wider than it is tall
        MapLeftCorner.X = diff;

}
//Distance in km, Acceleration in m/s^2, Speed in km/h, Time in seconds
float maxAccelerationDistance(float acceleration, float maxspeed)
{
    float Tm = maxspeed / acceleration;
    float d = (acceleration * Tm * Tm) / 2;
    return d;
}

float timeToPoint(float distance, float acceleration, float maxspeed)
{
    if (distance / 2 < maxAccelerationDistance(acceleration, maxspeed))
    {
        float time = 2 * (sqrt(2 * (distance/2) / acceleration));
        return time;
    }
    else
    {
        float time = 2*(maxspeed/acceleration);
        float remainingDistance = distance - 2 * maxAccelerationDistance(acceleration, maxspeed);
        time = time + (remainingDistance / maxspeed);
        return time;
    }
}
Point TranslateToScreen(MapPoint MapCoordinate)
{
    return Point(MapLeftCorner.X + MapCoordinate.first * MapRatio, MapLeftCorner.Y + MapCoordinate.second * MapRatio);
}

MapPoint TranslateToMap(Point ScreenCoordinate)
{
    //float margin = (MapSize - OuterCityDiameter) / 2;
    return MapPoint((ScreenCoordinate.X - MapLeftCorner.X) / MapRatio,
                    (ScreenCoordinate.Y - MapLeftCorner.Y) / MapRatio);
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    HWND                hWnd;
    MSG                 msg;
    WNDCLASS            wndClass;
    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR           gdiplusToken;

    // Initialize GDI+.
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    wndClass.style = CS_HREDRAW | CS_VREDRAW;
wndClass.lpfnWndProc = WndProc;
wndClass.cbClsExtra = 0;
wndClass.cbWndExtra = 0;
wndClass.hInstance = hInstance;
wndClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
wndClass.hCursor = LoadCursor(NULL, IDC_ARROW);
wndClass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
wndClass.lpszMenuName = NULL;
wndClass.lpszClassName = TEXT("GettingStarted");

RegisterClass(&wndClass);

hWnd = CreateWindow(
    TEXT("GettingStarted"),   // window class name
    TEXT("Getting Started"),  // window caption
    WS_OVERLAPPEDWINDOW,      // window style
    CW_USEDEFAULT,            // initial x position
    CW_USEDEFAULT,            // initial y position
    CW_USEDEFAULT,            // initial x size
    CW_USEDEFAULT,            // initial y size
    NULL,                     // parent window handle
    NULL,                     // window menu handle
    hInstance,                // program instance handle
    NULL);                    // creation parameters

ShowWindow(hWnd, SW_MAXIMIZE);
GetMapSize(hWnd);
UpdateWindow(hWnd);

while (GetMessage(&msg, NULL, 0, 0))
{
    TranslateMessage(&msg);
    DispatchMessage(&msg);
}

GdiplusShutdown(gdiplusToken);
return msg.wParam;
}


void DrawStop(Graphics& graphics, Point center, Color color)
{
    Pen      pen(Color(255, 0, 0, 0));
    pen.SetWidth(2);
    
    center.X -= lineWidth / 2;
    center.Y -= lineWidth / 2;

    graphics.DrawEllipse(&pen, center.X, center.Y, lineWidth - 2, lineWidth - 2);
    SolidBrush brush(color);
    graphics.FillEllipse(&brush, center.X, center.Y, lineWidth - 3, lineWidth - 3);
}

int CalculateLineStops(float L, float d)
{
    double numofstations = L / d;

    double ceilx = ceil(numofstations);
    double floorx = floor(numofstations);

    double valceil = abs(d - (L / (ceilx)));
    double valfloor = abs(d - (L / (floorx)));

    if (valceil < valfloor) numofstations = ceilx;
    else numofstations = floorx;

    return numofstations;
}

vector<MapPoint> GetInnerBounds(Graphics& graphics, MapPoint mapStart, MapPoint mapEnd, float stopLength, Color lineColor, Color stopColor, float innerL)
{
    Point start = TranslateToScreen(mapStart);
    Point end = TranslateToScreen(mapEnd);
    float L = OuterCityDiameter;
    

    // Calculate the outer line segment length
    float outerL = (L - innerL);

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(innerL, stopLength/4);

    // Calculate the delta values for drawing lines and stops
    int deltax = end.X - start.X;
    int deltay = end.Y - start.Y;
    vector<MapPoint> innerpoints;

    //coordinates of the inner city points
    Point innerbound1(start.X + (deltax * ((0.5 * (L - innerL)) / L)), start.Y + (deltay * ((0.5 * (L - innerL)) / L)));
    MapPoint converted1 = TranslateToMap(innerbound1);
    innerpoints.push_back(converted1);
    Point innerbound2(end.X - (deltax * ((0.5 * (L - innerL)) / L)), end.Y - (deltay * ((0.5 * (L - innerL)) / L)));
    MapPoint converted2 = TranslateToMap(innerbound2);
    innerpoints.push_back(converted2);
    return (innerpoints);
}

vector<Point> DrawStraightLine(Graphics& graphics, Point start, Point end, float stopLength, Color lineColor, Color stopColor)
{
    float lineLength = sqrt(pow(start.X - end.X, 2) + pow(start.Y - end.Y, 2)) / MapRatio;

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(lineLength, stopLength);

    // Calculate the delta values for drawing lines and stops
    int deltax = end.X - start.X;
    int deltay = end.Y - start.Y;

    Pen pen(lineColor);
    pen.SetWidth(lineWidth);
    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);

    // Draw the main line
    graphics.DrawLine(&pen, start.X, start.Y, end.X, end.Y);
    std::vector<Point> points;
    for (int i = 0; i <= totalstations; i++)
    {
        Point currentPoint(start.X + (deltax * ((float)(i) / (float)totalstations)), start.Y + (deltay * ((float)(i) / (float)totalstations)));
        DrawStop(graphics, currentPoint, stopColor);
        points.push_back(currentPoint);
    }
    return points;
}
vector<Point> drawCircle(HDC hdc, float stepAngle, float stopLength, float diameter)
{
    const float fullRotation = 360.0;
    Graphics graphics(hdc);
    Point center(MapSize / 2, MapSize / 2);
    int currentcolor = 0;
    int counter = 0;
    int numoftimes = 180 / stepAngle;
    vector<MapPoint> allinnerbounds1;
    vector<MapPoint> allinnerbounds2;

    for (float currentAngle = 0; currentAngle < fullRotation / 2.0; currentAngle += stepAngle)
    {

        float length = (OuterCityDiameter) / 2.0;
        float radians = (currentAngle) * (3.1415926 / 180.0);
        float cosine = cos(radians);
        float sine = sin(radians);

        int startX = center.X - round(length * cosine);
        int startY = center.Y - round(length * sine);
        int endX = center.X + round(length * cosine);
        int endY = center.Y + round(length * sine);

        int deltax = endX - startX;
        int deltay = endY - startY;

        vector<MapPoint> innerbounds = (GetInnerBounds(graphics, MapPoint((startX), (startY)), MapPoint((endX), (endY)), stopLength, subwayLanes[currentcolor].color, Color(255, 255, 255, 255), diameter));

        MapPoint innerbound1 = innerbounds[0];
        MapPoint innerbound2 = innerbounds[1];
        allinnerbounds1.push_back(innerbounds[0]);
        allinnerbounds2.push_back(innerbounds[1]);

        counter++;
    }
    vector<MapPoint> allinnerbounds;
    for (int i = 0; i < allinnerbounds1.size(); i++)
        allinnerbounds.push_back(allinnerbounds1[i]);
    for (int i = 0; i < allinnerbounds2.size(); i++)
        allinnerbounds.push_back(allinnerbounds2[i]);
    allinnerbounds.push_back(allinnerbounds1[0]);
    vector<Point> dotsoncircle;
    for (int i = 0; i < allinnerbounds.size() - 1; i++)
    {

        vector<Point> dots = DrawStraightLine(graphics, TranslateToScreen(allinnerbounds[i]), TranslateToScreen(allinnerbounds[i + 1]), stopLength, Color(255, 0, 0, 0), Color(255, 255, 255, 255));
        std::this_thread::sleep_for(10ms);
        dotsoncircle.insert(dotsoncircle.end(), dots.begin() + 1, dots.end());
    }
    return dotsoncircle;
}
vector<vector<Point>> PaintCity(HDC hdc, float stepAngle)
{
    vector<vector<Point>> linedots;
    const float fullRotation = 360.0;

    Point center(MapSize / 2, MapSize / 2); 
    std::vector<std::vector<Point>> linesandstops;

    Graphics graphics(hdc);

    
    int currentcolor = 0;
    int counter = 0;
    int numoftimes = 180 / stepAngle;
    vector<MapPoint> allinnerbounds1;
    vector<MapPoint> allinnerbounds2;

    for (float currentAngle = 0; currentAngle < fullRotation / 2.0; currentAngle += stepAngle)
    {
        vector<Point> CollectionofLinePoints;

        float length = (OuterCityDiameter) / 2.0;
        float radians = (currentAngle) * (3.1415926 / 180.0);
        float cosine = cos(radians);
        float sine = sin(radians);

        int startX = center.X - round(length * cosine);
        int startY = center.Y - round(length * sine);
        int endX = center.X + round(length * cosine);
        int endY = center.Y + round(length * sine);
        
        //This function draws all the points on the first outer secton
        int deltax = endX - startX;
        int deltay = endY - startY;

        float lineLength = sqrt(pow(startX - endX, 2) + pow(startY - endY, 2));

        //Gets inner bounds only
        vector<MapPoint> innerbounds = (GetInnerBounds(graphics, MapPoint((startX), (startY)), MapPoint((endX), (endY)), InnerCityStopLength, subwayLanes[currentcolor].color, Color(255, 255, 255, 255), InnerCityDiameter));

        MapPoint innerbound1 = innerbounds[0];
        MapPoint innerbound2 = innerbounds[1];
        allinnerbounds1.push_back(innerbounds[0]);
        allinnerbounds2.push_back(innerbounds[1]);

        vector<Point> outerlinepoints1 = DrawStraightLine(graphics, TranslateToScreen(MapPoint(startX, startY)), TranslateToScreen(innerbound1), OuterCityStopLength, subwayLanes[currentcolor].color, Color(255, 255, 255, 255));
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), outerlinepoints1.begin(), outerlinepoints1.end());

        
        vector<Point> innerlinepoints = DrawStraightLine(graphics, TranslateToScreen(innerbound1), TranslateToScreen(innerbound2), InnerCityStopLength, subwayLanes[currentcolor].color, Color(255, 255, 255, 255));

        for (int i = 0; i < innerlinepoints.size(); i++)
        {
            CollectionofLinePoints.push_back(innerlinepoints[i]);
        }


        vector<Point> outerlinepoints2 = DrawStraightLine(graphics, TranslateToScreen(innerbound2), TranslateToScreen(MapPoint(endX, endY)), OuterCityStopLength, subwayLanes[currentcolor].color, Color(255, 255, 255, 255));
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), outerlinepoints2.begin(), outerlinepoints2.end());
        //linesandstops.push_back(points);
        if (currentcolor != 15) currentcolor++;
        else currentcolor = 0;
        counter++;
        linedots.push_back(CollectionofLinePoints);
    }

    Pen pen(Color(255, 0, 0, 0));
    pen.SetWidth(lineWidth);

    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);
   
    vector<Point> innercircle = drawCircle(hdc, 45.0, InnerCityStopLength, InnerCityDiameter);
    linedots.push_back(innercircle);
    int numberofrings = 1;
    vector<Point> outercircle = drawCircle(hdc, 45.0, OuterCityStopLength/2, OuterCityDiameter/2);
    linedots.push_back(outercircle);
   
    for (int i = 0; i < linedots.size(); i++)
    {
        vector<MapPoint> subwayLine;

        for (int j = 0; j < linedots[i].size(); j++)
            subwayLine.push_back(TranslateToMap(linedots[i][j]));

        SubwayMap.push_back(subwayLine);
    }
 
    
    for (vector<MapPoint> line : SubwayMap)
        for (MapPoint stop : line)
        {
            std::this_thread::sleep_for(10ms);
            DrawStop(graphics, TranslateToScreen(stop), Color(128, 128, 128, 128));
            std::this_thread::sleep_for(10ms);
            DrawStop(graphics, TranslateToScreen(stop), Color(128, 255, 0, 0));
        }

    std::fstream fs;
    fs.open("points.txt", std::fstream::in | std::fstream::out | std::fstream::app);

    try
    {
        for (int i = 0; i < linedots.size(); i++)
        {
            if (i < linedots.size() - 1)
            {
                fs << "Points for line " << i + 1 << ":" << endl;
                for (int j = 0; j < linedots[i].size(); j++)
                {
                    fs << linedots[i][j].X << ", " << linedots[i][j].Y << endl;
                }
            }
            else
            {
                fs << "Points for circle line:" << endl;
                for (int j = 0; j < linedots[i].size(); j++)
                {
                    fs << linedots[i][j].X << ", " << linedots[i][j].Y << endl;
                }
            }
        }
    }
    catch (...)
    {
        fs.close();
    }
    //acceleration is m/s^2, top speed is m/s
    float timetest = timeToPoint(1000, 1, 30);
    return linedots;
} 


//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_TESTLINE));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_TESTLINE);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // Store instance handle in our global variable

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // Parse the menu selections:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        GetMapSize(hWnd);
        vector<vector<Point>> linepoints =  PaintCity(hdc, RadialLineStep);
        EndPaint(hWnd, &ps);
    }
    break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
