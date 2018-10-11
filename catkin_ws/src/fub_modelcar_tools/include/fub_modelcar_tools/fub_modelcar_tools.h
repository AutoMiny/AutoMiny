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



/////////////////////////////////////////////////////////////
// IO pair for steerings
//
//
class SteeringCalibrationData
{
private:
    friend class boost::serialization::access;

    // friend std::ostream & operator<<(std::ostream &os, const SteeringCalibrationData &p);
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
    SteeringCalibrationData(){};
    SteeringCalibrationData(int c, float r, float s,float f) :command(c), raduis(r), steering(s), feedback(f){}
    // copy constructor
    SteeringCalibrationData( const SteeringCalibrationData & rhs ):command(rhs.command), raduis(rhs.raduis), steering(rhs.steering), feedback(rhs.feedback){}

    // destructor
    ~SteeringCalibrationData(){}

	////////////////////////////
	/// \brief SteeringCalibrationData::operator =
	/// \param rhs
	/// \return
	///
	const SteeringCalibrationData & operator =(const SteeringCalibrationData & rhs){
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


class SpeedCalibrationData {

 public:
    SpeedCalibrationData() = default;
    SpeedCalibrationData(int c, double t):command(c), twist(t){}
    // destructor
    ~SpeedCalibrationData(){}
    // copy constructor
    SpeedCalibrationData( const SpeedCalibrationData & rhs ):command(rhs.command), twist(rhs.twist){}
    // ::operator =
    const SpeedCalibrationData & operator =(const SpeedCalibrationData & rhs){
        if(this != &rhs){ //Standard alias test
            command = rhs.command;
            twist= rhs.twist;
        }
        return *this;
    }
    int command;
    double twist;

 private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(command);
        ar & BOOST_SERIALIZATION_NVP(twist);
    }
};

class CameraCalibrationData {

 public:
    CameraCalibrationData() = default;
    CameraCalibrationData(cv::Mat i,cv::Mat d,double p,double h):intrinsicMatrix(i),distCoeffs(d), pitch(p), height(h){}
    // destructor
    ~CameraCalibrationData(){}
    // copy constructor
    CameraCalibrationData( const CameraCalibrationData & rhs ):intrinsicMatrix(rhs.intrinsicMatrix),
                                                                distCoeffs(rhs.distCoeffs),
                                                                pitch(rhs.pitch), 
                                                                height(rhs.height){}
    // ::operator =
    const CameraCalibrationData & operator =(const CameraCalibrationData & rhs){
        if(this != &rhs){ //Standard alias test
            intrinsicMatrix=rhs.intrinsicMatrix;
            pitch=rhs.pitch;
            height=rhs.height;
        }
        return *this;
    }
    cv::Mat intrinsicMatrix;
    cv::Mat distCoeffs;
    double pitch;
    double height;

 private:
    friend class boost::serialization::access;


    template<class Archive>
    void serialize(Archive &ar, cv::Mat& mat, const unsigned int version)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar & BOOST_SERIALIZATION_NVP(cols);
        ar & BOOST_SERIALIZATION_NVP(rows);
        ar & BOOST_SERIALIZATION_NVP(type);
        ar & BOOST_SERIALIZATION_NVP(continuous);

        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            size_t const data_size(rows * cols * mat.elemSize());
            ar & boost::serialization::make_array(mat.ptr(), data_size);
        } else {
            size_t const row_size(cols * mat.elemSize());
            for (int i = 0; i < rows; i++) {
                ar & boost::serialization::make_array(mat.ptr(i), row_size);
            }
        }
    }
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        serialize(ar, intrinsicMatrix, version);
        serialize(ar, distCoeffs, version);
        ar & BOOST_SERIALIZATION_NVP(pitch);
        ar & BOOST_SERIALIZATION_NVP(height);
    }

};



    template <class Obj>
    void saveXML(std::vector<Obj> & myPair, const char *fileName)
    {
        // make an archive, put myPair in an XML file fileName
        std::ofstream ofs(fileName);
        assert(ofs.good());
        boost::archive::xml_oarchive oa(ofs);
        oa << BOOST_SERIALIZATION_NVP(myPair);

    }

    template <class Obj>
    void restoreXML(std::vector<Obj> & myPair, const char *fileName)
    {
        std::ifstream ifs(fileName);
        if (ifs.fail())
        {
            std::cout<<"file " <<fileName << " opening failed";
        }
        else
        {
            assert(ifs.good());
            boost::archive::xml_iarchive ia(ifs);
            ia >> BOOST_SERIALIZATION_NVP(myPair);
        }
    }
    

    // void saveXML(CameraCalibrationData & myPair, const char *fileName);
    // void restoreXML(CameraCalibrationData & myPair, const char *fileName);
}
