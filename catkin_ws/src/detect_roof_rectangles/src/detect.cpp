#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>
#include <map>

ros::Publisher *pubPtr;

using namespace std;

typedef map<string, cmvision::Blob>::iterator it_type;

void cmvisionBlobsReceived(const cmvision::Blobs& blobsIn) {
	cmvision::Blobs blobsOut;

	map<string, cmvision::Blob> m;

	for (int i = 0; i < blobsIn.blob_count; ++i) {
		// TODO: rule to drop false positives?
		m[blobsIn.blobs[i].name] = blobsIn.blobs[i];
	}
    blobsOut.blob_count=m.size();
    
	for (it_type iterator = m.begin(); iterator != m.end(); ++iterator) {
		blobsOut.blobs.push_back(iterator->second);
		ROS_INFO_STREAM(
				"blob: \n name = " << iterator->second.name << "\n x = " << iterator->second.x << "\n y = " << iterator->second.y << "\n area = " << iterator->second.area << "\n top = " << iterator->second.top << "\n bottom = " << iterator->second.bottom << "\n left = " << iterator->second.left << "\n right = " << iterator->second.right);
	}

	pubPtr->publish(blobsOut);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "detect");

	ros::NodeHandle nh;

	pubPtr = new ros::Publisher(
			nh.advertise<cmvision::Blobs>("blobs/filtered",
					1000));

	ros::Subscriber sub = nh.subscribe("blobs", 1000, &cmvisionBlobsReceived);

	ros::spin();

	delete pubPtr;
}
