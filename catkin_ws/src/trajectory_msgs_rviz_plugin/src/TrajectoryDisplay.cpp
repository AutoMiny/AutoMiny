#include "TrajectoryDisplay.h"

namespace rviz {

    TrajectoryDisplay::TrajectoryDisplay() {
        lineWidth = new FloatProperty("Trajectory width", 0.02, "The width, in meters, of each path line.",
                                      this, SLOT(updateLineWidth()), this);
        lineWidth->setMin(0.001);

        alpha = new FloatProperty("Alpha", 1.0, "Path alpha value", this);
        alpha->setMin(0.0);
        alpha->setMax(1.0);

        colorOffset = new FloatProperty("Color Offset", 0.0, "Offset hue value of path trajectory color", this);
        colorOffset->setMin(0.0);
        colorOffset->setMax(1.0);

        maxVelocity = new FloatProperty("Max Velocity", 2.5, "Value for max velocity (green trajectory)", this);
        maxVelocity->setMin(0.0);
        bufferLength = new IntProperty("Buffer Length", 1, "Number of paths to display.", this,
                                       SLOT(updateBufferLength()));
        bufferLength->setMin(1);

        offset = new VectorProperty("Offset", Ogre::Vector3::ZERO,
                                    "Allows you to offset the path from the origin of the reference frame.", this,
                                    SLOT(updateOffset()));
    }

    TrajectoryDisplay::~TrajectoryDisplay() {
        destroyObjects();
    }

    void TrajectoryDisplay::onInitialize() {
        MFDClass::onInitialize();
        updateBufferLength();
    }

    void TrajectoryDisplay::reset() {
        MFDClass::reset();
        updateBufferLength();
    }

    void TrajectoryDisplay::updateLineWidth() {
        for (auto line : path) {
            if (line)
                line->setLineWidth(lineWidth->getFloat());
        }
        context_->queueRender();
    }

    void TrajectoryDisplay::updateOffset() {
        scene_node_->setPosition(offset->getVector());
        context_->queueRender();
    }

    void TrajectoryDisplay::destroyObjects() {
        for (auto &billboard_line : path) {
            if (billboard_line) {
                delete billboard_line;
                billboard_line = nullptr;
            }
        }
    }

    void TrajectoryDisplay::updateBufferLength() {
        destroyObjects();
        path.resize(bufferLength->getInt());
        for (auto &i : path) {
            auto *line = new rviz::BillboardLine(scene_manager_, scene_node_);
            i = line;
        }
    }

    bool validateFloats(autominy_msgs::msg::TrajectoryConstPtr const &msg) {
        bool valid = true;
        for (const autominy_msgs::msg::TrajectoryPoint &point : msg->trajectory) {
            valid = valid && validateFloats(point.pose);
        }
        return valid;
    }

    void TrajectoryDisplay::processMessage(autominy_msgs::msg::TrajectoryConstPtr const &msg) {
        size_t bufferIndex = messages_received_ % bufferLength->getInt();

        rviz::BillboardLine *pathLine = nullptr;
        pathLine = path[bufferIndex];
        pathLine->clear();

        if (!validateFloats(msg)) {
            setStatus(StatusProperty::Error, "Topic", "Message contained invalid floating point values");
            return;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
            RCLCPP_DEBUG(get_logger(), "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
                      qPrintable(fixed_frame_));
        }

        Ogre::Matrix4 transform(orientation);
        transform.setTrans(position);

        Ogre::ColourValue color;
        color.a = alpha->getFloat();

        pathLine->setNumLines(1);
        pathLine->setMaxPointsPerLine(msg->trajectory.size());
        pathLine->setLineWidth(lineWidth->getFloat());

        for (const autominy_msgs::msg::TrajectoryPoint &point : msg->trajectory) {
            auto pos = point.pose.position;
            auto xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
            tf::Vector3 vel;
            tf::vector3MsgToTF(point.velocity.linear, vel);
            float scaled_vel = std::min(vel.length() / maxVelocity->getFloat(), 1.0);
            color.setHSB((0.25 * scaled_vel) + colorOffset->getFloat(), 1, 1);
            pathLine->addPoint(xpos, color);
        }

    }
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz::TrajectoryDisplay, rviz::Display)
