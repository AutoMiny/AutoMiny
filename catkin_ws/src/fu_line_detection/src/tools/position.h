/**
 * @file position.h
 *
 * Definition of the position classes.
 *
 * We have
 *         * absolute positions on the field (view coordinate system) in cm
 *         * relative positions corresponding to the current robot position
 *             (where the robot is placed in (0,0)),
 *         * image positions, which determines the position in the image in pixel and
 *         * robot positions, which are positions with the angle of vision of the robot
 *
 * The field and its coordinate system
 *
 *      y
 *      ^       ______________________
 *      |    M  |          |          |  O
 *      |    Y  |_ -x, y   |   x, y  _|  P
 *      |    G  | |        |        | |  P
 * 0    +    O  | |       ( )       | |  G
 *      |    A  |_|        |        |_|  O
 *      |    L  |  -x,-y   |   x,-y   |  A
 *      |       |__________|__________|  L
 *      |
 *      -------------------+--------------> x
 *                         0
 *
 *
 * In the relative coordinate system, x points towards and y points to the left.
 * In the polar coordinates, the angle of zero points towards, positive angles to the left
 * and negative to the right (range [-180, 180])
 *
 *
 * @{
 */

#ifndef __POSITION_H__
#define __POSITION_H__

#include "messages/msg_position.pb.h"
#include "messages/msg_position.pb.h"

#include "utils/math/Math.h"
#include "utils/math/Common.h"
#include "utils/units.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <typeinfo>


/*------------------------------------------------------------------------------------------------*/

class Position;
class PositionImage;
class PositionRelative;
class PositionAbsolute;
class PositionRobot;
class Polar;
class SphericalCoordinate;


/*------------------------------------------------------------------------------------------------*/

/**
 ** "Abstract" position class
 */

class Position {
protected:
	int16_t x;
	int16_t y;

	Position(int16_t _x=SHRT_MAX, int16_t _y=SHRT_MAX) :
		x(_x), y(_y) {
	}

public:
	virtual ~Position() {
		x = SHRT_MAX;
		y = SHRT_MAX;
	}

	/**
	 * Gets the x value
	 * @return x
	 */
	inline int16_t getX() const {
		return x;
	}

	/**
	 * Gets the y value
	 * @return y
	 */
	inline int16_t getY() const {
		return y;
	}

	/**
	 * Sets the position
	 * @param _x new x coordinate
	 * @param _y new y coordinate
	 */
	inline void setPosition(int16_t _x, int16_t _y) {
		x = _x;
		y = _y;
	}

	/**
	 * Sets the x coordinate
	 * @param _x
	 */
	inline void setX(int16_t _x) {
		x = _x;
	}

	/**
	 * Sets the y coordinate
	 * @param _y
	 */
	inline void setY(int16_t _y) {
		y = _y;
	}

	/**
	 *
	 * @param _x
	 * @param _y
	 */
	inline void setValues(int16_t _x, int16_t _y) {
		x = _x;
		y = _y;
	}

	/**
	 *
	 * @param _x
	 */
	inline void addX(int16_t _x) {
		x += _x;
	}

	/**
	 *
	 * @param _x
	 */
	inline void addY(int16_t _y) {
		y += _y;
	}

	/**
	 * Gets the distance between this position and position p
	 * @param p Pointer on the other position
	 * @return euclidic distance or 65535 iff this and p aren't from the same type
	 */
	virtual uint16_t getDistance(const Position &p) const {
		// if same class
		if (typeid(*this) == typeid(p)) {
			return (uint16_t) round((sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y))));
		}
		else {
			// this object and p aren't from the same subclass, so it's impossible to calculate the distance
			return 65535;
		}
	}

	virtual float distanceToModel(const Position &m1, const Position &m2) const;

	virtual void rotateAsVectorByAngle(Degree alpha);
	virtual void rotateAsVectorByAngle(Radian alpha);

	virtual bool operator==(const Position &p) const {
		return this->x == p.x && this->y == p.y;
	}

	virtual bool operator!=(const Position &p) const {
		return this->x != p.x || this->y != p.y;
	}

	virtual bool isValid() const {
		return x != SHRT_MAX && y != SHRT_MAX;
	}

	virtual void setInvalid() {
		x = SHRT_MAX;
		y = SHRT_MAX;
	}
};



/*------------------------------------------------------------------------------------------------*/

/**
 * Defines position of object in the image with pixel coordinates.
 */

class PositionImage: public Position {
public:
	PositionImage(int16_t _x = SHRT_MAX, int16_t _y = SHRT_MAX) :
		Position(_x, _y) {
	}

	virtual ~PositionImage() {
	}

	// proto serialization
	virtual de::fumanoids::message::Vec2i asProtoPosImg() const;
};


/*------------------------------------------------------------------------------------------------*/

/**
 * Defines absolute position on the field
 */

class PositionAbsolute: public Position {
public:
	PositionAbsolute(int16_t _x = SHRT_MAX, int16_t _y = SHRT_MAX) :
		Position(_x, _y) {
	}

	virtual ~PositionAbsolute() {
	}

	/// explicit transformation
	PositionRelative translateToRelative(const PositionRobot &robotPos) const;
	PositionRelative translateToRelative(const arma::colvec& robotPose) const;
	SphericalCoordinate translateToSpherical(const arma::colvec& robotPose) const;

	// TODO: can we really add/subtract absolute positions? Or shouldn't
	//       this be adding/subtracting a relative (or polar) position
	//       to/from an absolute position only?

	PositionAbsolute operator+ (const PositionAbsolute &pos) const {
		return PositionAbsolute(x + pos.x, y + pos.y);
	}
	PositionAbsolute operator- (const PositionAbsolute &pos) const {
		return PositionAbsolute(x - pos.x, y - pos.y);
	}
	PositionAbsolute& operator+= (const PositionAbsolute &pos) {
		x += pos.x;
		y += pos.y;
		return *this;
	}
	PositionAbsolute& operator-= (const PositionAbsolute &pos) {
		x -= pos.x;
		y -= pos.y;
		return *this;
	}

	PositionAbsolute getMiddleOf(const PositionAbsolute &B) {
		PositionAbsolute pAbs;

		int16_t dx = (this->getX() + B.getX()) / 2,
				dy = (this->getY() + B.getY()) / 2;

		pAbs.setX(dx);
		pAbs.setY(dy);

		return pAbs;
	}

	// proto conversion
	PositionAbsolute(const de::fumanoids::message::Position &pos)
		: Position(SHRT_MAX, SHRT_MAX)
	{
		if (pos.positiontype() == de::fumanoids::message::Position_PositionType_ABSOLUTE) {
			x = pos.x();
			y = pos.y();
		}
	}

	de::fumanoids::message::Position getAsProtobuf() const
	{
		de::fumanoids::message::Position pos;

		pos.set_positiontype(de::fumanoids::message::Position_PositionType_ABSOLUTE);
		pos.set_x(getX());
		pos.set_y(getY());

		return pos;
	}

	bool isOnField(int16_t puffer = 0) const;
};


/*------------------------------------------------------------------------------------------------*/
/**
 ** Defines a relative position.
 **
 ** This means, that the robot coordinates are (0,0) and
 ** the object of interest is placed corresponding to that point
 ** the robot is looking along the positive x-axis
 ** the y-axis is directed to the left
 */
class PositionRelative: public Position {
public:
	PositionRelative(int16_t _x=SHRT_MAX, int16_t _y=SHRT_MAX)
		: Position(_x, _y)
	{}
	PositionRelative(const de::fumanoids::message::Position &pos);
	virtual ~PositionRelative() {}

	// transformation of coordinates
	PositionAbsolute translateToAbsolute(const PositionRobot& robotPos) const;
	PositionAbsolute translateToAbsolute(double absX, double absY, double rotInRad) const;

	Polar getAsPolar() const;
	SphericalCoordinate getAsSpherical() const;

	/// get angle
	inline Radian getAngle() const {
		return atan2(y, x) * radians;
	}

	// misc
	PositionRelative rotateBy90() const;
	PositionRelative getMiddleOf(const PositionRelative &B) const;
	int16_t getDistanceToMyself() const;

	// proto stuff
	de::fumanoids::message::Position getAsProtobuf() const;
};


/*------------------------------------------------------------------------------------------------*/

/**
 ** Defines the position of a robot with the angle of view
 */

class PositionRobot: public PositionAbsolute {
private:
	/// angle of view
	Degree angle;

public:
	PositionRobot(int16_t _x = SHRT_MAX, int16_t _y = SHRT_MAX)
		: PositionAbsolute(_x, _y)
		, angle(SHRT_MAX*degrees)
	{
	}

	PositionRobot(int16_t _x, int16_t _y, Radian _angle)
		: PositionAbsolute(_x, _y)
		, angle(Degree(Math::normalize(_angle)))
	{
	}

	PositionRobot(int16_t _x, int16_t _y, Degree _angle)
		: PositionAbsolute(_x, _y)
		, angle(Math::normalize(_angle))
	{
	}

	PositionRobot(const PositionAbsolute &pos, Degree _angle)
		: PositionAbsolute(pos.getX(), pos.getY())
		, angle(_angle)
	{
	}

	PositionRobot(const PositionAbsolute &pos, Radian _angle)
		: PositionAbsolute(pos.getX(), pos.getY())
		, angle(Degree(_angle))
	{
	}

	PositionRobot(const Pose2D &pos)
		: PositionAbsolute((int16_t)pos.translation.x, (int16_t)pos.translation.y)
		, angle(Degree(Math::normalize(pos.getAngle())))
	{
	}

	virtual ~PositionRobot() {
	}

	Degree getAngle() const {
		return angle;
	}

	void setAngle(Degree _angle) {
		angle = _angle;
	}

	void setAngle(Radian _angle) {
		angle = Degree(_angle);
	}
};


/*------------------------------------------------------------------------------------------------*/

/**
 ** Polar coordinates
 */
class Polar {
private:
	int16_t r; /// Distance to point
	int16_t theta; /// angle to point

public:
	Polar(int16_t _r=SHRT_MAX, int16_t _theta=SHRT_MAX) : r(_r), theta(_theta) {}
	virtual ~Polar() {}

	void setR(int16_t _r) {
		r = _r;
	}

	int16_t getR() const{
		return r;
	}

	void setTheta(int16_t _theta) {
		theta = _theta;
	}

	int16_t getTheta() const {
		return theta;
	}

	bool isValid() const {
		return r != SHRT_MAX && theta != SHRT_MAX;
	}

	bool isLeftFrom(const Polar &p) const {

		int16_t tT = getTheta(),
				pT = p.getTheta();

		//case 1: both position are behind us and one has a positive, while the other one has a negative angle
		if (abs(tT)>45 && abs(pT)>45 &&
				// (tT<0&&pT>0) xor (tT>0&&pT<0):
				(((tT<0&&pT>0)&&(!(tT>0&&pT<0)))||((!(tT<0&&pT>0))&&(tT>0&&pT<0)))) {
			return tT < pT;
		}
		//case 2: regular
		else {
			return tT > pT;
		}

	}

	PositionRelative getAsRelative() const;
};

/*------------------------------------------------------------------------------------------------*/

/**
 * SphericalCoordinate represents points through pitch and yaw.
 *
 * If the robot looks straight ahead the pitch and yaw is 0.
 *
 * See tests/testSphericCoordinates.cpp for the expected behavior.
 *
 * See: "On Sensor Model Design Choices for Humanoid Robot Localization"
 * by Stefan Tasse, et al.
 */

class SphericalCoordinate
{
private:
	double pitch; /// Pitch
	double yaw; /// Yaw

public:
	SphericalCoordinate()
		: pitch(0.f)
		, yaw(0.f)
	{}

	SphericalCoordinate(double pitchInRad, double yawInRad)
		: pitch(pitchInRad)
		, yaw(yawInRad)
	{}

	SphericalCoordinate(Radian pitch, Radian yaw)
		: pitch(pitch.value())
		, yaw(yaw.value())
	{}

	virtual ~SphericalCoordinate() {}

	PositionRelative getAsRelative() const;

	void setPitchInRad(double pitchInRad) {
		pitch = pitchInRad;
	}

	double getPitchInRad() const {
		return pitch;
	}

	double getPitchInDeg() const {
//		return Math::toDegrees(pitch);
//		TODO: toDegrees() is deprecated, but this function isn't used in our project at the moment
		  return 999.999;
	}

	Radian getPitch() const {
		return pitch * radians;
	}

	void setYawInRad(double yawInRad) {
		yaw = yawInRad;
	}

	double getYawInRad() const {
		return yaw;
	}
	double getYawInDeg() const {
//		return Math::toDegrees(yaw);
//		TODO: toDegrees() is deprecated, but this function isn't used in our project at the moment
		  return 999.999;
	}

	Radian getYaw() const {
		return yaw * radians;
	}

	bool isValid() const {
		return pitch > 0 && pitch <= Math::pi_2;
	}

	void invalidate() {
		pitch = 0.f;
	}

	SphericalCoordinate operator- (const SphericalCoordinate& other) const
	{
		return SphericalCoordinate(Math::normalize((pitch - other.pitch) * radians),
		                           Math::normalize((yaw - other.yaw) * radians));
	}
};


/*------------------------------------------------------------------------------------------------*/

class PositionTranslator {
public:
	PositionTranslator(const PositionRobot& pos);
	virtual ~PositionTranslator() {};
	PositionAbsolute translate(const PositionRelative& p) const;
private:
	float sinTheta, cosTheta;
	int16_t dx, dy;
};

/**
 * @}
 */


#endif /* __POSITION_H__ */
