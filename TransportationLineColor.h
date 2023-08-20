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
    wstring name;

    TransportationLineColor(Color col, wstring nm);
};

