#pragma once

#include <autominy_msgs/Trajectory.h>
#include <boost/bind.hpp>
#include <tf/transform_listener.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/message_filter_display.h"

namespace rviz {
    class FloatProperty;

    class IntProperty;

    class BillboardLine;

    class VectorProperty;

/**
 * \class TrajectoryDisplay
 * \brief Displays an autominy::Trajectory message
 */
    class TrajectoryDisplay : public MessageFilterDisplay<autominy_msgs::Trajectory> {
    Q_OBJECT
    public:
        TrajectoryDisplay();

        ~TrajectoryDisplay() override;

        /** @brief Overridden from Display. */
        void reset() override;

    protected:
        /** @brief Overridden from Display. */
        void onInitialize() override;

        /** @brief Overridden from MessageFilterDisplay. */
        void processMessage(autominy_msgs::TrajectoryConstPtr const &msg) override;

    private Q_SLOTS:

        void updateBufferLength();

        void updateLineWidth();

        void updateOffset();

    private:
        void destroyObjects();

        std::vector<rviz::BillboardLine *> path;

        FloatProperty *maxVelocity;
        FloatProperty *colorOffset;
        FloatProperty *alpha;
        FloatProperty *lineWidth;
        IntProperty *bufferLength;
        VectorProperty *offset;
    };
}
