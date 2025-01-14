#ifndef IEDGEINFO_H
#define IEDGEINFO_H

struct IEdgeInfo
{
    virtual double GetWeight() const = 0;
    virtual ~IEdgeInfo() = default;
};

struct BasicEdgeInfo : public IEdgeInfo
{
    double distance;

    BasicEdgeInfo(double dist = 0.0)
        : distance(dist) {}

    double GetWeight() const override
    {
        return distance;
    }
};

struct PenaltyEdgeInfo : public IEdgeInfo
{
    double distance;
    double penalty;

    PenaltyEdgeInfo(double dist = 0.0, double p = 0.0)
        : distance(dist), penalty(p) {}

    double GetWeight() const override
    {
        return distance + penalty;
    }
};

#endif
