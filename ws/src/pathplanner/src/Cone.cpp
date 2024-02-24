#include "path_planner/Cone.h"

Cone::Cone(double x, double y, Cone::TrackSide side)
    : m_x{x}
    , m_y{y}
    , m_side{side}
{}

double Cone::GetX() const 
{ 
    return m_x; 
}

void Cone::SetX(double x) 
{ 
    m_x = x; 
}

double Cone::GetY() const 
{ 
    return m_y; 
}

void Cone::SetY(double y) 
{ 
    m_y = y; 
}

Cone::TrackSide Cone::GetSide() const 
{ 
    return m_side; 
}

void Cone::SetSide(Cone::TrackSide side) 
{ 
    m_side = side; 
}