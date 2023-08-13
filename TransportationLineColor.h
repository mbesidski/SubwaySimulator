#pragma once

#include <objidl.h>
#include <gdiplus.h>
#include <iostream>

using namespace std;
using namespace Gdiplus;

class TransportationLineColor
{
public:

    Color color;
    string name;

    TransportationLineColor(Color col, string nm);
    //TransportationLineColor(const TransportationLineColor& sl);
};

