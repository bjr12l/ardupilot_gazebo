/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#pragma once

#include <gst/gst.h>
#include <gz/msgs.hh>
#include <gz/plugin/Plugin.hh>
#include <gz/rendering/Camera.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim.hh>
#include <gz/sim/System.hh>
#include <gz/transport.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/SuppressWarning.hh>

#include <memory>
#include <mutex>
#include <string>

namespace gz {

class GstCameraPluginPrivate;

namespace sim {
namespace systems {

/**
 * @class GstCameraPlugin
 * A Gazebo plugin that can be attached to a camera and then streams the video data using gstreamer.
 * It streams to a configurable UDP IP and UDP Port, defaults are respectively 127.0.0.1 and 5600.
 *
 * Connect to the stream via command line with:
 * gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
 *  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
 */

class GZ_SIM_VISIBLE GstCameraPlugin final : public System,
                                             public ISystemConfigure,
                                             public ISystemPostUpdate {
   public:
    GstCameraPlugin();
    virtual ~GstCameraPlugin() = default;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) final;
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) final;

   private:
    std::unique_ptr<GstCameraPluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz
