/*
 * Lanes.cpp
 *
 *  Created on: 04.02.2015
 *      Author: conrad
 */

#include "LanePolynomial.h"

LanePolynomial::LanePolynomial() {
    lanesDetected = false;
};

LanePolynomial::~LanePolynomial() {}

const NewtonPolynomial& LanePolynomial::getLanePoly() const {
    return lanePoly;
}

const std::vector<FuPoint<int>>& LanePolynomial::getPoints() const {
    return lanePoints;
}

bool LanePolynomial::hasDetected() const {
    return lanesDetected;
}

void LanePolynomial::setLanePoly(NewtonPolynomial newLanePoly) {
    lanePoly = newLanePoly;
}

void LanePolynomial::setPoints(std::vector<FuPoint<int>> newPoints) {
    lanePoints = newPoints;
}

void LanePolynomial::setNotDetected() {
    lanesDetected = false;
    lanePoly.clear();
}

void LanePolynomial::setDetected() {
    lanesDetected = true;
}

const ePosition LanePolynomial::getLastUsedPosition() const {
    return lastUsedPosition;
}

void LanePolynomial::setLastUsedPosition(ePosition position) {
    lastUsedPosition = position;
}
