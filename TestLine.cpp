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
float InnerCityStopLength = OuterCityStopLength/4;
int lineWidth = 20;

typedef pair<float, float> MapPoint;

vector<vector<MapPoint>> SubwayMap;

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

vector<Point> DrawOuterLine1(Graphics& graphics, MapPoint mapStart, MapPoint mapEnd, float stopLength,  Color lineColor, Color stopColor)
{
    Point start = TranslateToScreen(mapStart);
    Point end = TranslateToScreen(mapEnd);
    float L = OuterCityDiameter;
    float innerL = InnerCityDiameter;

    // Calculate the outer line segment length
    float outerL = (L - innerL);

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops((L-innerL)/2, stopLength);

    // Calculate the delta values for drawing lines and stops
    int deltax = end.X - start.X;
    int deltay = end.Y - start.Y;

    Pen pen(lineColor);
    pen.SetWidth(lineWidth);

    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);

    std::vector<Point> points;

    //coordinates of the inner city points
    Point innerbound1(start.X + (deltax * ((0.5 * (L - innerL)) / L)), start.Y + (deltay * ((0.5 * (L - innerL)) / L)));
    
    
    Point innerbound2(end.X - (deltax * ((0.5 * (L - innerL)) / L)), end.Y - (deltay * ((0.5 * (L - innerL)) / L)));
    
    
    deltax = innerbound1.X - start.X;
    deltay = innerbound1.Y - start.Y;
    // Draw the main line
    graphics.DrawLine(&pen, start.X, start.Y, innerbound1.X, innerbound1.Y);
    
    for (int i = 0; i < totalstations; i++)
    {
        if (i == 0)
        {
            DrawStop(graphics, start, stopColor);
            points.push_back(start);
        }
        Point currentPoint(start.X + (deltax * ((float)(i+1)/(float)totalstations)), start.Y + (deltay * ((float)(i+1) / (float)totalstations)));
        DrawStop(graphics, currentPoint, stopColor);
        points.push_back(currentPoint);
    }

    return points;
}

vector<vector <Point>> DrawInnerLineandInnerBounds(Graphics& graphics, MapPoint mapStart, MapPoint mapEnd, float stopLength, Color lineColor, Color stopColor)
{
    Point start = TranslateToScreen(mapStart);
    Point end = TranslateToScreen(mapEnd);
    float L = OuterCityDiameter;
    float innerL = InnerCityDiameter;

    // Calculate the outer line segment length
    float outerL = (L - innerL);

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(innerL, stopLength/4);

    // Calculate the delta values for drawing lines and stops
    int deltax = end.X - start.X;
    int deltay = end.Y - start.Y;

    Pen pen(lineColor);
    pen.SetWidth(lineWidth);

    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);

    vector<Point> innerpoints;

    //coordinates of the inner city points
    Point innerbound1(start.X + (deltax * ((0.5 * (L - innerL)) / L)), start.Y + (deltay * ((0.5 * (L - innerL)) / L)));
    innerpoints.push_back(innerbound1);
    Point innerbound2(end.X - (deltax * ((0.5 * (L - innerL)) / L)), end.Y - (deltay * ((0.5 * (L - innerL)) / L)));
    innerpoints.push_back(innerbound2);

    // Draw the main line
    graphics.DrawLine(&pen, innerbound1.X, innerbound1.Y, innerbound2.X, innerbound2.Y);

    deltax = innerbound2.X - innerbound1.X;
    deltay = innerbound2.Y - innerbound1.Y;
    vector<Point> points;

    for (int i = 0; i < totalstations; i++)
    {
        if (i == 0) DrawStop(graphics, start, stopColor);
        Point currentPoint(innerbound1.X + (deltax * ((float)(i + 1) / (float)totalstations)), innerbound1.Y + (deltay * ((float)(i + 1) / (float)totalstations)));
        DrawStop(graphics, currentPoint, stopColor);
        points.push_back(currentPoint);
    }
    vector<vector<Point>> allpoints;
    allpoints.push_back(points); allpoints.push_back(innerpoints);
    return allpoints;
}

vector<Point> DrawOuterLine2(Graphics& graphics, MapPoint mapStart, MapPoint mapEnd, float stopLength, Color lineColor, Color stopColor)
{
    Point start = TranslateToScreen(mapStart);
    Point end = TranslateToScreen(mapEnd);

    // Calculate the outer line segment length
    float outerL = OuterCityDiameter - InnerCityDiameter;

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops((OuterCityDiameter - InnerCityDiameter) / 2, stopLength);

    // Calculate the delta values for drawing lines and stops
    int deltax = end.X - start.X;
    int deltay = end.Y - start.Y;

    Pen pen(lineColor);
    pen.SetWidth(lineWidth);

    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);

    //coordinates of the inner city points
    Point innerbound1(start.X + (deltax * ((0.5 * (OuterCityDiameter - InnerCityDiameter)) / OuterCityDiameter)), start.Y + (deltay * ((0.5 * (OuterCityDiameter - InnerCityDiameter)) / OuterCityDiameter)));
    Point innerbound2(end.X - (deltax * ((0.5 * (OuterCityDiameter - InnerCityDiameter)) / OuterCityDiameter)), end.Y - (deltay * ((0.5 * (OuterCityDiameter - InnerCityDiameter)) / OuterCityDiameter)));

    // Draw the main line
    graphics.DrawLine(&pen, innerbound2.X, innerbound2.Y, end.X, end.Y);
    deltax = end.X - innerbound2.X;
    deltay = end.Y - innerbound2.Y;
    vector<Point> points;
    for (int i = 0; i < totalstations; i++)
    {
        if (i == 0) DrawStop(graphics, start, stopColor);
        Point currentPoint(innerbound2.X + (deltax * ((float)(i + 1) / (float)totalstations)), innerbound2.Y + (deltay * ((float)(i + 1) / (float)totalstations)));
        DrawStop(graphics, currentPoint, stopColor);
        points.push_back(currentPoint);
    }
   //Draws inner stops
    DrawStop(graphics, innerbound1, Color(255, 255, 0, 255));
    DrawStop(graphics, innerbound2, Color(255, 255, 0, 255));
    return points;
}
vector<Point> DrawStraightLine(Graphics& graphics, Point mapStart, Point mapEnd, float stopLength, float L, Color lineColor, Color stopColor)
{
    Point start = mapStart;
    Point end = mapEnd;

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(L, stopLength);

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
    for (int i = 0; i < totalstations; i++)
    {
        if (i == 0) DrawStop(graphics, start, stopColor);
        Point currentPoint(start.X + (deltax * ((float)(i + 1) / (float)totalstations)), start.Y + (deltay * ((float)(i + 1) / (float)totalstations)));
        DrawStop(graphics, currentPoint, stopColor);
        points.push_back(currentPoint);
    }
    return points;
}

vector<vector<Point>> PaintCity(HDC hdc, float stepAngle)
{
    vector<vector<Point>> linedots;
    const float fullRotation = 360.0;

    Point center(MapSize / 2, MapSize / 2); 
    std::vector<std::vector<Point>> linesandstops;

    Graphics graphics(hdc);
    
    vector<Color> colors;
    colors.push_back(Color(255, 255, 0, 0));        // Bright Red
    colors.push_back(Color(255, 255, 165, 0));      // Bright Orange
    colors.push_back(Color(255, 255, 255, 0));      // Bright Yellow
    colors.push_back(Color(255, 0, 255, 0));        // Bright Green
    colors.push_back(Color(255, 0, 255, 255));      // Bright Blue
    colors.push_back(Color(255, 255, 0, 255));      // Bright Magenta
    colors.push_back(Color(255, 0, 0, 0));          // Black
    colors.push_back(Color(255, 255, 128, 0));      // Bright Orange-Red
    colors.push_back(Color(255, 255, 255, 128));    // Bright Yellow-Light
    colors.push_back(Color(255, 128, 255, 128));    // Bright Lime Green
    colors.push_back(Color(255, 128, 128, 255));    // Bright Periwinkle
    colors.push_back(Color(255, 255, 128, 255));    // Bright Orchid
    colors.push_back(Color(255, 128, 255, 255));    // Bright Turquoise
    colors.push_back(Color(255, 255, 0, 128));      // Bright Hot Pink
    colors.push_back(Color(255, 128, 0, 255));      // Bright Indigo
    colors.push_back(Color(255, 0, 0, 255));        // Bright Light Blue
    
    int currentcolor = 0;
    int counter = 0;
    int numoftimes = 180 / stepAngle;
    vector<Point> allinnerbounds1;
    vector<Point> allinnerbounds2;
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
        vector<Point> outerlinepoints1 = DrawOuterLine1(graphics, MapPoint((startX), (startY)), MapPoint((endX), (endY)), OuterCityStopLength, colors[currentcolor], Color(255, 255, 255, 255));
        for (int i = 0; i < outerlinepoints1.size() ; i++)
        {
            CollectionofLinePoints.push_back(outerlinepoints1[i]);
        }

        //beside drawing the points on the first section, this function gives the inner bounds. What I do next is I push the points into the organized vector
        vector<vector<Point>> innerpointsandinnerbounds = DrawInnerLineandInnerBounds(graphics, MapPoint((startX), (startY)), MapPoint((endX), (endY)), OuterCityStopLength, colors[currentcolor], Color(255, 255, 255, 255));
        vector<Point> innerlinepoints = innerpointsandinnerbounds[0];
        vector<Point> innerbounds = innerpointsandinnerbounds[1];
        for (int i = 0; i < innerlinepoints.size(); i++)
        {
            CollectionofLinePoints.push_back(innerlinepoints[i]);
        }
        allinnerbounds1.push_back(innerbounds[0]);
        allinnerbounds2.push_back(innerbounds[1]);

        //This function draws all the points on the first outer secton
        vector<Point> outerlinepoints2 = DrawOuterLine2(graphics, MapPoint((startX), (startY)), MapPoint((endX), (endY)), OuterCityStopLength, colors[currentcolor], Color(255, 255, 255, 255));
        for (int i = 0; i < outerlinepoints2.size(); i++)
        {
            CollectionofLinePoints.push_back(outerlinepoints2[i]);
        }
        //linesandstops.push_back(points);
        if (currentcolor != 15) currentcolor++;
        else currentcolor = 0;
        counter++;
        linedots.push_back(CollectionofLinePoints);
    }
    vector<Point> allinnerbounds;
    for (int i = 0; i < allinnerbounds1.size(); i++)
        allinnerbounds.push_back(allinnerbounds1[i]);
    for (int i = 0; i < allinnerbounds2.size(); i++)
        allinnerbounds.push_back(allinnerbounds2[i]);
    allinnerbounds.push_back(allinnerbounds1[0]);

    Pen pen(Color(255, 0, 0, 0));
    pen.SetWidth(lineWidth);

    pen.SetEndCap(LineCapRound);
    pen.SetStartCap(LineCapRound);
    float L = sqrt(pow((allinnerbounds[1].X - allinnerbounds[0].X), 2) + pow((allinnerbounds[1].Y - allinnerbounds[0].Y), 2)) / MapRatio;
    vector<Point> dotsoncircle;
    for (int i = 0; i < allinnerbounds.size()-1; i++)
    {
        vector<Point> dots = DrawStraightLine(graphics, allinnerbounds[i], allinnerbounds[i + 1], InnerCityStopLength, L, Color(255, 0, 0, 0), Color(255, 255, 255, 255));
        for (int i = 0; i < dots.size() ; i++)
        {
            dotsoncircle.push_back(dots[i]);
        }
    }
    linedots.push_back(dotsoncircle);

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
            std::this_thread::sleep_for(100ms);
            DrawStop(graphics, TranslateToScreen(stop), Color(128, 128, 128, 128));
            std::this_thread::sleep_for(100ms);
            DrawStop(graphics, TranslateToScreen(stop), Color(128, 255, 0, 0));
        }

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
