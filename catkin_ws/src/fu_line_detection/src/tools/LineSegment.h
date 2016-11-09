/*
 * LineSegment.h
 *
 *  Created on: Apr 7, 2014
 *      Author: 'Jannis Ihrig'
 */

#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include "FuPoint.h"

template<typename T>
class LineSegment
{
public:
	LineSegment(FuPoint<T> start, FuPoint<T> end):
		start(start),
		end(end)
	{}
	virtual ~LineSegment() {};

	FuPoint<T> getStart() const {
		return start;
	}
	FuPoint<T> getEnd() const {
		return end;
	}
private:
	FuPoint<T> start;
	FuPoint<T> end;
};

#endif /* LINESEGMENT_H_ */
