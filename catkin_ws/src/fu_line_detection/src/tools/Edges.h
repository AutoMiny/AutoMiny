
#ifndef EDGES_H_
#define EDGES_H_

#include "FuPoint.h"

#include <vector>
#include <stdint.h>
#include <limits.h>

class EdgePoint
{
public:
	EdgePoint():
		m_child(NULL),
		m_parent(NULL),
		m_scoreToChild(INFINITY),
		m_scoreToParent(INFINITY)
	{

	}

	EdgePoint(const FuPoint<int> &posImg, const FuPoint<Meter> &relPos, const int16_t val):
		m_child(NULL),
		m_parent(NULL),
		m_scoreToChild(INFINITY),
		m_scoreToParent(INFINITY)
	{
		m_imgPos = posImg;
		m_relPos = relPos;
		m_value = val;
	}

	EdgePoint(const EdgePoint &i_edgePoint)
	{
		m_imgPos = i_edgePoint.m_imgPos;
		m_relPos = i_edgePoint.m_relPos;
		m_value = i_edgePoint.m_value;

		m_child = i_edgePoint.m_child;
		m_parent = i_edgePoint.m_parent;
		m_scoreToChild = i_edgePoint.m_scoreToChild;
		m_scoreToParent = i_edgePoint.m_scoreToParent;
	}

	virtual ~EdgePoint() {}

	inline const FuPoint<int> &getImgPos() const
	{
		return m_imgPos;
	}

	inline const FuPoint<Meter> &getRelPos() const
	{
		return m_relPos;
	}

	inline void setImgPos(const FuPoint<int> &pos)
	{
		m_imgPos = pos;
	}

	inline void setRelPos(const FuPoint<Meter> &pos)
	{
		m_relPos = pos;
	}

	inline bool isPositive() const
	{
		return (m_value > 0);
	}

	inline int16_t getValue() const
	{
		return m_value;
	}

	int operator<(const EdgePoint &i_edgePoint) const
	{
		FuPoint<int> imgPos = i_edgePoint.getImgPos();
		if ( m_imgPos.getX() == imgPos.getX() && m_imgPos.getY() < imgPos.getY()) return 1;
		if ( m_imgPos.getX() < imgPos.getX() ) return 1;
		return 0;
	}

	/* this is used to connect edges to a lane marking */
	EdgePoint *m_child, *m_parent;
	float m_scoreToChild, m_scoreToParent;
	int flag = 0;

private:
	FuPoint<int> m_imgPos;
	FuPoint<Meter> m_relPos;
	/* the value of this edge (positive indicates change from black to white and vice versa) */
	int16_t m_value;
};

class Edges
{
public:
	inline void setEdgeListFront(std::vector<std::vector<EdgePoint>> list)
	{
		m_edgePointsFront = std::move(list);
	}

	inline const std::vector<std::vector<EdgePoint>>& getFrontList() const { return m_edgePointsFront; }

	std::vector<EdgePoint> getAllList() const
	{
		std::vector<EdgePoint> allList;
		for (auto &scanline : m_edgePointsFront)
		{
			for (auto &edgePoint : scanline)
			{
				allList.push_back(edgePoint);
			}
		}
		return allList;
	}

private:
	/* the first edgeList in the EdgeListList is the closest. Followed by the next and so on... */
	std::vector<std::vector<EdgePoint>> m_edgePointsFront; /* all edges from left to right in front of us */
};

/* same thing as a list of edge list but in this case sorted by the angle to the relative coordinates of the car in degrees */
typedef std::vector<std::vector<EdgePoint>> EdgeListListAngular;


/*------------------------------------------------------------------------------------------------*/

#endif /* EDGES_H_ */
