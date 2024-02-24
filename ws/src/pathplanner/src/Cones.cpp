#include "path_planner/Cones.h"
#include <math.h>

void Cones::AddCone(double x, double y, Cone::TrackSide side)
{
    Cone cone(x, y, side);
    if (side == Cone::TrackSide::INNER)
    {
        m_innerCones.push_back(cone);
    }
    else if (side == Cone::TrackSide::OUTER)
    {
        m_outerCones.push_back(cone);
    }
}

const Cone &Cones::GetCone(Cone::TrackSide side, int index) const
{
    if (side == Cone::TrackSide::INNER)
    {
        return m_innerCones.at(index);
    }
    else if (side == Cone::TrackSide::OUTER)
    {
        return m_outerCones.at(index);
    }
}

void Cones::RemoveCone(Cone::TrackSide side, int index)
{
    if (side == Cone::TrackSide::INNER)
    {
        m_innerCones.erase(m_innerCones.begin() + index);
    }
    else if (side == Cone::TrackSide::OUTER)
    {
        m_outerCones.erase(m_outerCones.begin() + index);
    }
}

int Cones::GetNumCones(Cone::TrackSide side) const
{
    if (side == Cone::TrackSide::INNER)
    {
        return m_innerCones.size();
    }
    else if (side == Cone::TrackSide::OUTER)
    {
        return m_outerCones.size();
    }
    return -1;
}

void Cones::GenerateTestingMap()
{
    for (Cone cone : m_testingConesInner)
    {
        m_innerCones.push_back(cone);
    }
    for (Cone cone : m_testingConesOuter)
    {
        m_outerCones.push_back(cone);
    }
}

void Cones::CalculateWayPoints(Cones cones)
{
    for (int i = 0; i < m_innerCones.size(); i++)
    {
        wayPoint point;

        double dx;
        double dy;
        dx = cones.GetCone(Cone::TrackSide::INNER, i).GetX() - cones.GetCone(Cone::TrackSide::OUTER, i).GetX();
        dy = cones.GetCone(Cone::TrackSide::INNER, i).GetY() - cones.GetCone(Cone::TrackSide::OUTER, i).GetY();
        point.heading = atan2(dy, dx) - M_PI / 2;

        point.x = (cones.GetCone(Cone::TrackSide::INNER, i).GetX() + cones.GetCone(Cone::TrackSide::OUTER, i).GetX()) / 2;
        point.y = (cones.GetCone(Cone::TrackSide::INNER, i).GetY() + cones.GetCone(Cone::TrackSide::OUTER, i).GetY()) / 2;
        
        wayPoints.push_back(point);
        // if (i < m_innerCones.size() - 1)
        // {
        //     dx = cones.GetCone(Cone::TrackSide::INNER, i + 1).GetX() - cones.GetCone(Cone::TrackSide::OUTER, i).GetX();
        //     dy = cones.GetCone(Cone::TrackSide::INNER, i + 1).GetY() - cones.GetCone(Cone::TrackSide::OUTER, i).GetY();
        //     point.heading = atan2(dy, dx) - M_PI / 2;

        //     point.x = (cones.GetCone(Cone::TrackSide::INNER, i).GetX() + cones.GetCone(Cone::TrackSide::OUTER, i + 1).GetX()) / 2;
        //     point.y = (cones.GetCone(Cone::TrackSide::INNER, i).GetY() + cones.GetCone(Cone::TrackSide::OUTER, i + 1).GetY()) / 2;
        //     wayPoints.push_back(point);
        // }
        // else
        // {
        //     dx = cones.GetCone(Cone::TrackSide::INNER, 0).GetX() - cones.GetCone(Cone::TrackSide::OUTER, i).GetX();
        //     dy = cones.GetCone(Cone::TrackSide::INNER, 0).GetY() - cones.GetCone(Cone::TrackSide::OUTER, i).GetY();
        //     point.heading = atan2(dy, dx) - M_PI / 2;

        //     point.x = (cones.GetCone(Cone::TrackSide::INNER, 0).GetX() + cones.GetCone(Cone::TrackSide::OUTER, i).GetX()) / 2;
        //     point.y = (cones.GetCone(Cone::TrackSide::INNER, 0).GetY() + cones.GetCone(Cone::TrackSide::OUTER, i).GetY()) / 2;
        //     wayPoints.push_back(point);
        // }
    }
}