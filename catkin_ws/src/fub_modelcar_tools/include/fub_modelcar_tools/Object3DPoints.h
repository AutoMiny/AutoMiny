#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
// include headers that implement a archive in XML format

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <opencv2/opencv.hpp>


namespace fub_modelcar_tools {

    class Object3DPoints
    {
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar, const unsigned int)
        {
            ar & BOOST_SERIALIZATION_NVP(point3d.x);
            ar & BOOST_SERIALIZATION_NVP(point3d.y);
            ar & BOOST_SERIALIZATION_NVP(point3d.z);
        }

    public:
        Object3DPoints(){};
        Object3DPoints(cv::Point3f p) :point3d(p){}
        // destructor
        ~Object3DPoints(){}
        // copy constructor
        Object3DPoints( const Object3DPoints & rhs ):point3d(rhs.point3d){}
    	// ::operator =
    	const Object3DPoints & operator =(const Object3DPoints & rhs){
    	    if(this != &rhs){ //Standard alias test
                point3d=rhs.point3d;
    	    }
    	    return *this;
    	}
        cv::Point3f point3d;
    };

}
