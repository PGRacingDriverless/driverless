class Cone 
{
    public:
        enum TrackSide { INNER, OUTER };

        Cone(double x, double y, TrackSide side);

        double GetX() const;
        void SetX(double x);

        double GetY() const;
        void SetY(double y);

        TrackSide GetSide() const;
        void SetSide(TrackSide side);

    private:
        double m_x;
        double m_y;
        TrackSide m_side;
};