#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
// include headers that implement a archive in XML format

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <ros/ros.h>



namespace fub_modelcar_tools {



/////////////////////////////////////////////////////////////
// IO pair
//
//
class IO_pair
{
private:
    friend class boost::serialization::access;

    // friend std::ostream & operator<<(std::ostream &os, const IO_pair &p);
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(command);  //between 0 and 180 (90 is almost stright)
        ar & BOOST_SERIALIZATION_NVP(raduis); //between 0 and 20 meter
        ar & BOOST_SERIALIZATION_NVP(steering); //between -pi/2 and pi/2 radian
        ar & BOOST_SERIALIZATION_NVP(feedback); //between 200 and 450 //?!Uint16 : arduino ADC output!
    }

public:
    IO_pair(){};
    IO_pair(int c, float r, float s,float f) :command(c), raduis(r), steering(s), feedback(f){}
    // copy constructor
    IO_pair( const IO_pair & rhs ):command(rhs.command), raduis(rhs.raduis), steering(rhs.steering), feedback(rhs.feedback){}

    // destructor
    ~IO_pair(){}

	////////////////////////////
	/// \brief IO_pair::operator =
	/// \param rhs
	/// \return
	///
	const IO_pair & operator =(const IO_pair & rhs){
	    if(this != &rhs){ //Standard alias test
	        command = rhs.command;
            raduis= rhs.raduis;
	        steering= rhs.steering;
            feedback= rhs.feedback;
	    }
	    return *this;
	}
    int command;
    float raduis;
    float steering;
    float feedback;

};

    void saveXML(std::vector<IO_pair> & myPair, const char *fileName);
    void restoreXML(std::vector<IO_pair> & myPair, const char *fileName);
}
