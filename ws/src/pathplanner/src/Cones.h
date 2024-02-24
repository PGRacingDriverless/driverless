

#include <vector>
#include "path_planner/Cone.h"

struct wayPoint
{
    double x;
    double y;
    double heading;
};

    class Cones
{
public:
    void AddCone(double x, double y, Cone::TrackSide side);
    const Cone &GetCone(Cone::TrackSide side, int index) const;
    void RemoveCone(Cone::TrackSide side, int index);
    int GetNumCones(Cone::TrackSide side) const;
    void GenerateTestingMap();
    void CalculateWayPoints(Cones cones);
    std::vector<wayPoint> wayPoints;

private:
    std::vector<Cone> m_innerCones;
    std::vector<Cone> m_outerCones;
    std::vector<Cone> m_testingConesInner = {
        Cone(18, 30, Cone::TrackSide::INNER),
        Cone(21, 31, Cone::TrackSide::INNER),
        Cone(24, 31, Cone::TrackSide::INNER),
        Cone(28, 30, Cone::TrackSide::INNER),
        Cone(31, 30, Cone::TrackSide::INNER),
        Cone(36, 30, Cone::TrackSide::INNER),
        Cone(41, 30, Cone::TrackSide::INNER),
        Cone(43, 29, Cone::TrackSide::INNER),
        Cone(45, 27, Cone::TrackSide::INNER),
        Cone(45, 24, Cone::TrackSide::INNER),
        Cone(44, 22, Cone::TrackSide::INNER),
        Cone(40, 20, Cone::TrackSide::INNER),
        Cone(36, 18, Cone::TrackSide::INNER),
        Cone(32, 16, Cone::TrackSide::INNER),
        Cone(29, 17, Cone::TrackSide::INNER),
        Cone(27, 19, Cone::TrackSide::INNER),
        Cone(27, 22, Cone::TrackSide::INNER),
        Cone(25, 24, Cone::TrackSide::INNER),
        Cone(22, 25, Cone::TrackSide::INNER),
        Cone(19, 27, Cone::TrackSide::INNER),
    };

    std::vector<Cone> m_testingConesOuter = {
        Cone(13, 33, Cone::TrackSide::OUTER),
        Cone(18, 36, Cone::TrackSide::OUTER),
        Cone(24, 37, Cone::TrackSide::OUTER),
        Cone(28, 37, Cone::TrackSide::OUTER),
        Cone(31, 36, Cone::TrackSide::OUTER),
        Cone(36, 36, Cone::TrackSide::OUTER),
        Cone(40, 36, Cone::TrackSide::OUTER),
        Cone(45, 36, Cone::TrackSide::OUTER),
        Cone(54, 31, Cone::TrackSide::OUTER),
        Cone(52, 24, Cone::TrackSide::OUTER),
        Cone(49, 17, Cone::TrackSide::OUTER),
        Cone(43, 14, Cone::TrackSide::OUTER),
        Cone(39, 12, Cone::TrackSide::OUTER),
        Cone(35, 11, Cone::TrackSide::OUTER),
        Cone(29, 10, Cone::TrackSide::OUTER),
        Cone(22, 14, Cone::TrackSide::OUTER),
        Cone(21, 19, Cone::TrackSide::OUTER),
        Cone(19, 21, Cone::TrackSide::OUTER),
        Cone(16, 24, Cone::TrackSide::OUTER),
        Cone(13, 27, Cone::TrackSide::OUTER),
    };
};