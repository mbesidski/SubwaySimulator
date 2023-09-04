// TestLine.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "TestLine.h"
#include <cmath>
#include <numbers>
#include <vector>
#include <windows.h>
#include "TransportationLineColor.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#include "TransportationSystem.h"
#include "TransportationSystemAssumptions.h"
#include "DijkstraPath.h"


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
int lineWidth = 20;

int originStop = -1;
int destinationStop = -1;
bool bFullRepaint = true;

TransportationSystem transportationSystem;

vector<TransportationLineColor> subwayLines =
{
    {Color(255, 255, 0, 0), L"Bright Red" },
    {Color(255, 255, 165, 0), L"Bright Orange"},
    {Color(255, 255, 255, 0), L"Bright Yellow"},
    {Color(255, 0, 255, 0), L"Bright Green"},
    {Color(255, 0, 255, 255), L"Bright Blue"},
    {Color(255, 255, 0, 255), L"Bright Magenta"},
    {Color(255, 0, 0, 0), L"Black"},
    {Color(255, 255, 128, 0), L"Bright Orange-Red"},
    {Color(255, 255, 255, 128), L"Bright Yellow-Light"},
    {Color(255, 128, 255, 128), L"Bright Lime Green"},
    {Color(255, 128, 128, 255), L"Bright Periwinkle"},
    {Color(255, 255, 128, 255), L"Bright Orchid"},
    {Color(255, 128, 255, 255), L"Bright Turquoise"},
    {Color(255, 255, 0, 128), L"Bright Hot Pink"},
    {Color(255, 128, 0, 255), L"Bright Indigo"},
    {Color(255, 0, 0, 255), L"Bright Light Blue"}
};

void GetMapSize(HWND hWnd)
{
    RECT windowSize;
    GetClientRect(hWnd, &windowSize);
    float screenSize = min(windowSize.bottom - windowSize.top, windowSize.right - windowSize.left);
    
    MapRatio = screenSize / TransportationSystemAssumptions::MapSize;
    MapLeftCorner = Point(0, 0);
    
    int diff = abs(((windowSize.bottom - windowSize.top) - (windowSize.right - windowSize.left)) / 2);
    
    if (windowSize.bottom - windowSize.top > windowSize.right - windowSize.left) //taller than it is wide
        MapLeftCorner.Y = diff;
    else //wider than it is tall
        MapLeftCorner.X = diff;

}

Point TranslateToScreen(TransportationStopCoordinates MapCoordinate)
{
    return Point(MapLeftCorner.X + MapCoordinate.x * MapRatio, MapLeftCorner.Y + MapCoordinate.y * MapRatio);
}

TransportationStopCoordinates TranslateToMap(Point ScreenCoordinate)
{
    return TransportationStopCoordinates((ScreenCoordinate.X - MapLeftCorner.X) / MapRatio,
                    (ScreenCoordinate.Y - MapLeftCorner.Y) / MapRatio);
}

void GetCityPoints();

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

    GetCityPoints();

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


void DrawStop(Graphics& graphics, Point center, Color color, wstring label)
{
    Pen      pen(Color(255, 0, 0, 0));
    pen.SetWidth(2);
    
    center.X -= lineWidth / 2;
    center.Y -= lineWidth / 2;

    graphics.DrawEllipse(&pen, center.X, center.Y, lineWidth - 2, lineWidth - 2);
    SolidBrush brush(color);
    graphics.FillEllipse(&brush, center.X, center.Y, lineWidth - 3, lineWidth - 3);

    FontFamily  fontFamily(L"Times New Roman");
    Font        font(&fontFamily, 15, FontStyleRegular, UnitPixel);
    PointF      pointF(center.X - 1, center.Y + 1);
 
    SolidBrush  solidBrush(Color(255, 0, 0, 0));
    if (originStop >= 0)
        solidBrush.SetColor(Color(255, 255, 255, 255));

    graphics.DrawString(label.c_str(), -1, &font, pointF, &solidBrush);
}

int CalculateLineStops(float L, float d)
{
    float numofstations = L / d;

    float ceilx = ceil(numofstations);
    float floorx = floor(numofstations);

    float valceil = abs(d - (L / ceilx));
    float valfloor = abs(d - (L / floorx));

    if (valceil < valfloor) 
        numofstations = ceilx;
    else 
        numofstations = floorx;

    return numofstations;
}

vector<TransportationStopCoordinates> GetInnerBounds(TransportationStopCoordinates mapStart, TransportationStopCoordinates mapEnd, float stopLength, float innerL)
{
    TransportationStopCoordinates start = mapStart;
    TransportationStopCoordinates end = mapEnd;
    float L = TransportationSystemAssumptions::OuterCityDiameter;
    

    // Calculate the outer line segment length
    float outerL = (L - innerL);

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(innerL, stopLength / 4);

    // Calculate the delta values for drawing lines and stops
    float deltax = end.x - start.x;
    float deltay = end.y - start.y;
    vector<TransportationStopCoordinates> innerpoints;

    //coordinates of the inner city points
    TransportationStopCoordinates innerbound1(start.x + (deltax * ((0.5 * (L - innerL)) / L)), start.y + (deltay * ((0.5 * (L - innerL)) / L)));
    innerpoints.push_back(innerbound1);
    TransportationStopCoordinates innerbound2(end.x - (deltax * ((0.5 * (L - innerL)) / L)), end.y - (deltay * ((0.5 * (L - innerL)) / L)));
    innerpoints.push_back(innerbound2);

    return (innerpoints);
}

vector<TransportationStopCoordinates> GetStraightLinePoints(TransportationStopCoordinates start, TransportationStopCoordinates end, float stopLength)
{
    float lineLength = sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));

    // Calculate the number of stations in the outer and inner sections
    int totalstations = CalculateLineStops(lineLength, stopLength);

    // Calculate the delta values for drawing lines and stops
    float deltax = end.x - start.x;
    float deltay = end.y - start.y;


   vector<TransportationStopCoordinates> points;
    for (int i = 0; i <= totalstations; i++)
    {
        TransportationStopCoordinates currentPoint(start.x + (deltax * (i / (float)totalstations)), start.y + (deltay * (i / (float)totalstations)));
        points.push_back(currentPoint);
    }
    return points;
}

vector<TransportationStopCoordinates> getCirclePoints(float stopLength, float diameter)
{
    const float fullRotation = 360.0;
    
    int currentcolor = 0;
    int counter = 0;
    int numoftimes = 180 / TransportationSystemAssumptions::RadialLineStep;
    vector<TransportationStopCoordinates> allinnerbounds1;
    vector<TransportationStopCoordinates> allinnerbounds2;

    for (float currentAngle = 0; currentAngle < fullRotation / 2.0; currentAngle += TransportationSystemAssumptions::RadialLineStep)
    {

        float length = (TransportationSystemAssumptions::OuterCityDiameter) / 2.0;
        float radians = (currentAngle) * (3.1415926 / 180.0);
        float cosine = cos(radians);
        float sine = sin(radians);

        float startX = TransportationSystemAssumptions::MapCenter().x - length * cosine;
        float startY = TransportationSystemAssumptions::MapCenter().y - length * sine;
        float endX = TransportationSystemAssumptions::MapCenter().x + length * cosine;
        float endY = TransportationSystemAssumptions::MapCenter().y + length * sine;

        float deltax = endX - startX;
        float deltay = endY - startY;

        vector<TransportationStopCoordinates> innerbounds = (GetInnerBounds( TransportationStopCoordinates((startX), (startY)), TransportationStopCoordinates((endX), (endY)), stopLength, diameter));

        TransportationStopCoordinates innerbound1 = innerbounds[0];
        TransportationStopCoordinates innerbound2 = innerbounds[1];
        allinnerbounds1.push_back(innerbounds[0]);
        allinnerbounds2.push_back(innerbounds[1]);

        counter++;
    }
    vector<TransportationStopCoordinates> allinnerbounds;
    for (int i = 0; i < allinnerbounds1.size(); i++)
        allinnerbounds.push_back(allinnerbounds1[i]);
    for (int i = 0; i < allinnerbounds2.size(); i++)
        allinnerbounds.push_back(allinnerbounds2[i]);
    allinnerbounds.push_back(allinnerbounds1[0]);
    vector<TransportationStopCoordinates> dotsoncircle;
    for (int i = 0; i < allinnerbounds.size() -1; i++)
    {
        vector<TransportationStopCoordinates> dots = GetStraightLinePoints(allinnerbounds[i+1], allinnerbounds[i], stopLength);
        //this_thread::sleep_for(10ms);
        dotsoncircle.insert(dotsoncircle.begin(), dots.begin() + 1, dots.end());
    }
    return dotsoncircle;
}

void AddLineToSystem(vector<TransportationStopCoordinates> stops, TransportationLineColor line_color, bool bCircular)
{
    int line_id = transportationSystem.AddLine(TransportationLine(line_color, bCircular));
    for (TransportationStopCoordinates p : stops)
        transportationSystem.AddStop(TransportationStop(p, line_id));
}

void PaintPath(Graphics& graphics)
{
    if (originStop != -1)
    {
        if (destinationStop != -1)
        {
            wstring stop_label = transportationSystem.GetStopLabel(destinationStop);
            DrawStop(graphics, TranslateToScreen(transportationSystem.stops[destinationStop].mapCoordinates), Color(255, 0, 0, 0), stop_label);
            DijkstraPath path = transportationSystem.PathBetweenStops[{originStop, destinationStop}];
            
            Pen pen(Color(255, 255, 255, 255));
            pen.SetWidth(3);

            for (int i = 0; i < path.stops.size() - 1; i++)
            {
                Point start = TranslateToScreen(transportationSystem.stops[path.stops[i]].mapCoordinates);
                Point end = TranslateToScreen(transportationSystem.stops[path.stops[i + 1]].mapCoordinates);
                graphics.DrawLine(&pen, start.X, start.Y, end.X, end.Y);
            }
            // Draw the main line
            
            this_thread::sleep_for(10ms);
        }
        else
        {
            wstring stop_label = transportationSystem.GetStopLabel(originStop);
            DrawStop(graphics, TranslateToScreen(transportationSystem.stops[originStop].mapCoordinates), Color(255, 0, 0, 0), stop_label);
        }
    }
}
void GetCityPoints()
{
    //vector<vector<Point>> linedots;
    const float fullRotation = 360.0;

    TransportationStopCoordinates center(TransportationSystemAssumptions::MapSize / 2, TransportationSystemAssumptions::MapSize / 2);

    int counter = 0;
    int numoftimes = 180 / TransportationSystemAssumptions::RadialLineStep;
    vector<TransportationStopCoordinates> allinnerbounds1;
    vector<TransportationStopCoordinates> allinnerbounds2;
    vector<TransportationStopCoordinates> allmidbounds1;
    vector<TransportationStopCoordinates> allmidbounds2;
    for (float currentAngle = 0; currentAngle < fullRotation / 2.0; currentAngle += TransportationSystemAssumptions::RadialLineStep)
    {
        vector<TransportationStopCoordinates> CollectionofLinePoints;

        float length = (TransportationSystemAssumptions::OuterCityDiameter) / 2.0;
        float radians = (currentAngle) * (3.1415926 / 180.0);
        float cosine = cos(radians);
        float sine = sin(radians);

        float startX = center.x - length * cosine;
        float startY = center.y - length * sine;
        float endX = center.x + length * cosine;
        float endY = center.y + length * sine;


        float deltax = endX - startX;
        float deltay = endY - startY;

        float lineLength = sqrt(pow(startX - endX, 2) + pow(startY - endY, 2));

        //Gets inner bounds only
        vector<TransportationStopCoordinates> innerbounds = 
            GetInnerBounds(TransportationStopCoordinates(startX, startY), 
                TransportationStopCoordinates(endX, endY), 
                TransportationSystemAssumptions::InnerCityStopLength, 
                TransportationSystemAssumptions::InnerCityDiameter);

        vector<TransportationStopCoordinates> midbounds = 
            GetInnerBounds(TransportationStopCoordinates(startX, startY), 
                TransportationStopCoordinates(endX, endY), 
                TransportationSystemAssumptions::InnerCityStopLength, 
                TransportationSystemAssumptions::MiddleCityDiameter);

        TransportationStopCoordinates innerbound1 = innerbounds[0];
        TransportationStopCoordinates innerbound2 = innerbounds[1];
        allinnerbounds1.push_back(innerbounds[0]);
        allinnerbounds2.push_back(innerbounds[1]);

        TransportationStopCoordinates midbound1 = midbounds[0];
        TransportationStopCoordinates midbound2 = midbounds[1];
        allmidbounds1.push_back(midbounds[0]);
        allmidbounds2.push_back(midbounds[1]);
        
        vector<TransportationStopCoordinates> outerlinepoints1 = GetStraightLinePoints(TransportationStopCoordinates(startX, startY), midbound1, TransportationSystemAssumptions::OuterCityStopLength);
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), outerlinepoints1.begin(), outerlinepoints1.end());

        vector<TransportationStopCoordinates> midlinepoints1 = GetStraightLinePoints(midbound1, innerbound1, TransportationSystemAssumptions::MiddleCityStopLength);
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), midlinepoints1.begin(), midlinepoints1.end());

        vector<TransportationStopCoordinates> innerlinepoints = GetStraightLinePoints(innerbound1, innerbound2, TransportationSystemAssumptions::InnerCityStopLength);
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), innerlinepoints.begin(), innerlinepoints.end());

        vector<TransportationStopCoordinates> midlinepoints2 = GetStraightLinePoints(innerbound2, midbound2, TransportationSystemAssumptions::MiddleCityStopLength);
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), midlinepoints2.begin(), midlinepoints2.end());

        vector<TransportationStopCoordinates> outerlinepoints2 = GetStraightLinePoints(midbound2, TransportationStopCoordinates(endX, endY), TransportationSystemAssumptions::OuterCityStopLength);
        CollectionofLinePoints.insert(CollectionofLinePoints.end(), outerlinepoints2.begin(), outerlinepoints2.end());

        AddLineToSystem(CollectionofLinePoints, subwayLines[counter], false);

        counter++;
    }
    
    vector<TransportationStopCoordinates> innercircle = getCirclePoints(TransportationSystemAssumptions::InnerCityStopLength, TransportationSystemAssumptions::InnerCityDiameter);
    AddLineToSystem(innercircle, subwayLines[counter], true);

    counter++;
    int numberofrings = 1;
    vector<TransportationStopCoordinates> outercircle = getCirclePoints(TransportationSystemAssumptions::MiddleCityStopLength, TransportationSystemAssumptions::MiddleCityDiameter);
    AddLineToSystem(outercircle, subwayLines[counter], true);

    transportationSystem.CalculateSuportingInfo();

}

void PaintCity(HDC hdc)
{

    Graphics graphics(hdc);


    if (!bFullRepaint)
    {
        PaintPath(graphics);
        return;
    }

    int minTraffic = transportationSystem.TrafficMap[{0, 1}];
    int maxTraffic = transportationSystem.TrafficMap[{0, 1}];

    for (auto segment : transportationSystem.TrafficMap)
    {
        if (segment.second < 0)
            continue;

        maxTraffic = max(maxTraffic, segment.second);
        minTraffic = min(minTraffic, segment.second);
    }
    
    int currentcolor = 0;
    for (int i = 0; i < transportationSystem.lines.size(); i++)
    {
        Pen pen(transportationSystem.lines[i].color.color);
        pen.SetWidth(lineWidth);
        pen.SetEndCap(LineCapRound);
        pen.SetStartCap(LineCapRound);
        for (int j = 0; j < transportationSystem.lines[i].stops.size() - 1; j++)
        {
            // Draw the main line
            int stop_idx = transportationSystem.lines[i].stops[j];
            int next_idx = transportationSystem.lines[i].stops[j + 1];

            Point start = TranslateToScreen(transportationSystem.stops[stop_idx].mapCoordinates);
            Point end = TranslateToScreen(transportationSystem.stops[next_idx].mapCoordinates);
            if (TransportationSystemAssumptions::isHeatMap)
            {
                int x;
                if (transportationSystem.TrafficMap[{stop_idx, next_idx}] > transportationSystem.TrafficMap[{next_idx, stop_idx}])
                {
                    x = 50 + 205 * ((transportationSystem.TrafficMap[{stop_idx, next_idx}] - minTraffic) / (float)(maxTraffic - minTraffic));
                }
                else x = 50 + 205 * ((transportationSystem.TrafficMap[{next_idx, stop_idx}] - minTraffic) / (float)(maxTraffic - minTraffic));
                pen.SetColor(Color(255, x, 128, 128));
            }
           
            graphics.DrawLine(&pen, start.X, start.Y, end.X, end.Y);
           

            if (transportationSystem.lines[i].bCircular)
            {
                int first_idx = transportationSystem.lines[i].stops[0];
                int last_idx = transportationSystem.lines[i].stops[transportationSystem.lines[i].stops.size()-1];
                Point first = TranslateToScreen(transportationSystem.stops[first_idx].mapCoordinates);
                Point last = TranslateToScreen(transportationSystem.stops[last_idx].mapCoordinates);

                graphics.DrawLine(&pen, first.X, first.Y, last.X, last.Y);
            }
            
        }
        
        if (currentcolor != 15) currentcolor++;
        else currentcolor = 0;
        
    }

    for (TransportationLine line : transportationSystem.lines)
    {
        for (int stop_idx : line.stops)
        {
            wstring stop_label = transportationSystem.GetStopLabel(stop_idx);
            DrawStop(graphics, TranslateToScreen(transportationSystem.stops[stop_idx].mapCoordinates), Color(128, 128, 128, 128), stop_label);
        }
    }
    for (TransportationLine line : transportationSystem.lines)
    {
        for (int stop_idx : line.stops)
        {
            wstring stop_label = transportationSystem.GetStopLabel(stop_idx);

            std::this_thread::sleep_for(10ms);
            DrawStop(graphics, TranslateToScreen(transportationSystem.stops[stop_idx].mapCoordinates), Color(128, 255, 0, 0), stop_label);
            std::this_thread::sleep_for(10ms);
            DrawStop(graphics, TranslateToScreen(transportationSystem.stops[stop_idx].mapCoordinates), Color(255, 255, 255, 255), stop_label);
        }
    }

    PaintPath(graphics);

    FontFamily  fontFamily(L"Times New Roman");
    Font        font(&fontFamily, 22, FontStyleBold, UnitPixel);
    PointF      pointF(20, 20);

    SolidBrush  solidBrush(Color(255, 0, 0, 0));
    graphics.DrawString(transportationSystem.GetTotals().c_str(), -1, &font, pointF, &solidBrush);
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
        PaintCity(hdc);
        EndPaint(hWnd, &ps);
    }
    break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    
    case WM_LBUTTONDOWN:
    {
        Point pt;
        pt.X = LOWORD(lParam);
        pt.Y = HIWORD(lParam);

        int stop_idx = transportationSystem.FindClosestTransportartionStop(TranslateToMap(pt), 0.15);

        if (stop_idx >= 0) //we clicked on stop
        {
            if (originStop != -1 && destinationStop != -1) // start from scratch
            {
                originStop = stop_idx;
                bFullRepaint = true;
                RECT windowSize;
                GetClientRect(hWnd, &windowSize);
                InvalidateRect(hWnd, &windowSize, true);
                break;
            }

            if (originStop == -1)
            {
                originStop = stop_idx;
                bFullRepaint = false;
                RECT rc{ pt.X - 30, pt.Y - 30, pt.X + 30, pt.Y + 30 };
                InvalidateRect(hWnd, &rc, false);
            }
            else
            {
                if (stop_idx != originStop)
                {
                    destinationStop = stop_idx;
                    //repaint while also plotting a path between two points
                    bFullRepaint = false;
                    RECT windowSize;
                    GetClientRect(hWnd, &windowSize);
                    InvalidateRect(hWnd, &windowSize, false);
                }
            }
        }
        
    }
    break;
    case WM_RBUTTONDOWN:
    {
        Point pt;
        pt.X = LOWORD(lParam);
        pt.Y = HIWORD(lParam);

        int stop_idx = transportationSystem.FindClosestTransportartionStop(TranslateToMap(pt), 0.15);

        if (stop_idx >= 0) //we clicked on stop
            MessageBox(hWnd, transportationSystem.GetStopInfo(stop_idx, originStop).c_str(), L"Transportation Stop", 0);
        else
        {
            HDC hdc = GetDC(hWnd);
            COLORREF col = GetPixel(hdc, pt.X, pt.Y);
            
            for (TransportationLine line : transportationSystem.lines)
            {
                Color c(col);
                if (line.color.color.ToCOLORREF() == col)
                {
                    wstring msg = L"Line " + to_wstring(line.id) + L" " + line.color.name + L"\n";
                    msg += transportationSystem.GetLineInfo(line.id, TranslateToMap(pt));
                    MessageBox(hWnd, msg.c_str(), L"Transportation Line Info", 0);
                    break;
                }
            }

        }
    }
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
