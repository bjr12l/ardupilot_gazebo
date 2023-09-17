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

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/RenderingEvents.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Sensor.hh>

#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>

#include <math.h>
#include <time.h>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "GstCameraPlugin.hh"

using namespace cv;

// Register plugin
GZ_ADD_PLUGIN(gz::sim::systems::GstCameraPlugin, gz::sim::System,
              gz::sim::systems::GstCameraPlugin::ISystemConfigure,
              gz::sim::systems::GstCameraPlugin::ISystemPostUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::GstCameraPlugin, "GstCameraPlugin")

namespace gz {

class GstCameraPluginPrivate {
   public:
    GstCameraPluginPrivate();
    virtual ~GstCameraPluginPrivate();

    void startGstThread();
    void stopGstThread();
    void gstCallback(GstElement *appsrc);

    void cbVideoStream(const gz::msgs::Int32 &_msg);

    void startStreaming();
    void stopStreaming();

    bool InitCamera(rendering::CameraPtr cam);
    void OnImage(const msgs::Image &);

    unsigned int width, height, depth;
    float rate;
    std::string format;
    std::string udpHost;
    int udpPort;
    bool useRtmp;
    std::string rtmpLocation;
    bool useCuda;
    rendering::ScenePtr scene = nullptr;
    rendering::CameraPtr camera = nullptr;
    gz::sim::Entity entity;
    std::string cameraName;
    transport::Node cam_img_node;
    pthread_t mThreadId;
    const std::string mTopicName = "~/video_stream";
    bool mIsActive = false;
    GMainLoop *gst_loop = nullptr;
    GstElement *source = nullptr;
    bool initialized = false;
};

namespace sim {
namespace systems {

GstCameraPlugin::GstCameraPlugin() : dataPtr{new GstCameraPluginPrivate()} {}

void GstCameraPlugin::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &_eventMgr) {
    this->dataPtr->udpHost = "127.0.0.1";
    if (_sdf->HasElement("udpHost")) {
        this->dataPtr->udpHost = _sdf->Get<std::string>("udpHost");
    }

    this->dataPtr->udpPort = 5600;
    if (_sdf->HasElement("udpPort")) {
        this->dataPtr->udpPort = _sdf->Get<int>("udpPort");
    }
    gzwarn << "Streaming video to ip: " << this->dataPtr->udpHost
           << " port: " << this->dataPtr->udpPort << std::endl;

    if (_sdf->HasElement("rtmpLocation")) {
        this->dataPtr->rtmpLocation = _sdf->Get<std::string>("rtmpLocation");
        this->dataPtr->useRtmp = true;
    } else {
        this->dataPtr->useRtmp = false;
    }

    // Use CUDA for video encoding
    if (_sdf->HasElement("useCuda")) {
        this->dataPtr->useCuda = _sdf->Get<bool>("useCuda");
    } else {
        this->dataPtr->useCuda = false;
    }

    this->dataPtr->entity = _entity;

    // Listen to Gazebo topic
    /*this->dataPtr->node_handle_.
    Subscribe(this->dataPtr->mTopicName,
                         &gz::GstCameraPluginPrivate::cbVideoStream,
                         this->dataPtr.get());*/
}

void GstCameraPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                 const gz::sim::EntityComponentManager &_ecm) {
    // if (!_info.paused) {}

    // TODO: We need oneshot initialization
    if (this->dataPtr->initialized)
        return;

    // get scene
    if (!this->dataPtr->scene) {
        this->dataPtr->scene = rendering::sceneFromFirstRenderEngine();
        if (!this->dataPtr->scene)
            return;
    }

    // return if scene not ready or no sensors available.
    if (!this->dataPtr->scene->IsInitialized() ||
        this->dataPtr->scene->SensorCount() == 0) {
        return;
    }

    // get camera
    if (!this->dataPtr->camera) {
        if (this->dataPtr->cameraName.empty()) {
            this->dataPtr->cameraName = removeParentScope(
                scopedName(this->dataPtr->entity, _ecm, "::", false), "::");
        }
        auto sensor =
            this->dataPtr->scene->SensorByName(this->dataPtr->cameraName);
        if (!sensor) {
            gzerr << "Unable to find sensor: " << this->dataPtr->cameraName
                  << std::endl;
            return;
        }
        rendering::CameraPtr cam =
            std::dynamic_pointer_cast<rendering::Camera>(sensor);
        if (!cam) {
            gzerr << "Sensor: " << this->dataPtr->cameraName
                  << " is not a camera" << std::endl;
            return;
        }

        if (!this->dataPtr->InitCamera(cam))
            return;

        this->dataPtr->camera = cam;
    }

    auto cameraEntComp =
        _ecm.Component<components::Camera>(this->dataPtr->entity);
    if (!cameraEntComp)
        return;

    // get sensor topic
    sdf::Sensor sensorSdf = cameraEntComp->Data();
    std::string topic = sensorSdf.Topic();
    if (topic.empty()) {
        auto scoped = scopedName(this->dataPtr->entity, _ecm);
        topic = transport::TopicUtils::AsValidTopic(scoped + "/image");
        if (topic.empty()) {
            gzerr << "Failed to generate valid topic for entity [" << scoped
                  << "]" << std::endl;
            return;
        }
    }

    this->dataPtr->cam_img_node.Subscribe(
        topic, &GstCameraPluginPrivate::OnImage, this->dataPtr.get());

    this->dataPtr->initialized = true;

    // Start by default
    this->dataPtr->startStreaming();
}

}  // namespace systems
}  // namespace sim

bool GstCameraPluginPrivate::InitCamera(rendering::CameraPtr cam) {
    if (cam->ImageWidth() == 0 || cam->ImageHeight() == 0)
        return false;

    this->width = cam->ImageWidth();
    this->height = cam->ImageHeight();
    this->format = cam->ImageFormat();
    // TODO
    this->depth = 24;  // this->dataPtr->camera->ImageDepth();
    this->rate = 60;   // this->camera->RenderRate();

    gzwarn << "Init camera, width: " << this->width
           << " height: " << this->height << " framerate: " << this->rate
           << std::endl;

    if (!isfinite(this->rate)) {
        this->rate = 60.0;
    }

    return true;
}

static void *start_thread(void *param) {
    GstCameraPluginPrivate *plugin = (GstCameraPluginPrivate *)param;
    plugin->startGstThread();
    return nullptr;
}

void GstCameraPluginPrivate::startGstThread() {
    gst_init(nullptr, nullptr);

    this->gst_loop = g_main_loop_new(nullptr, FALSE);
    if (!this->gst_loop) {
        gzerr << "Create loop failed. \n";
        return;
    }

    GstElement *pipeline = gst_pipeline_new(nullptr);
    if (!pipeline) {
        gzerr << "ERR: Create pipeline failed. \n";
        return;
    }

    GstElement *source = gst_element_factory_make("appsrc", nullptr);
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);

    GstElement *encoder;
    if (useCuda) {
        encoder = gst_element_factory_make("nvh264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr);
    } else {
        encoder = gst_element_factory_make("x264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6,
                     "tune", 4, "key-int-max", 10, nullptr);
    }

    GstElement *payloader;
    GstElement *sink;

    if (useRtmp) {
        payloader = gst_element_factory_make("flvmux", nullptr);
        sink = gst_element_factory_make("rtmpsink", nullptr);
        g_object_set(G_OBJECT(sink), "location", this->rtmpLocation.c_str(),
                     nullptr);
    } else {
        payloader = gst_element_factory_make("rtph264pay", nullptr);
        sink = gst_element_factory_make("udpsink", nullptr);
        g_object_set(G_OBJECT(sink), "host", this->udpHost.c_str(), "port",
                     this->udpPort, nullptr);
    }

    if (!source || !queue || !converter || !encoder || !payloader || !sink) {
        gzerr << "ERR: Create elements failed. \n";
        return;
    }

    // Configure source element
    g_object_set(
        G_OBJECT(source), "caps",
        gst_caps_new_simple(
            "video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT,
            this->width, "height", G_TYPE_INT, this->height, "framerate",
            GST_TYPE_FRACTION, (unsigned int)this->rate, 1, nullptr),
        "is-live", TRUE, "do-timestamp", TRUE, "stream-type",
        GST_APP_STREAM_TYPE_STREAM, "format", GST_FORMAT_TIME, nullptr);

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
                     payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder, payloader,
                              sink, nullptr) != TRUE) {
        gzerr << "ERR: Link all the elements failed. \n";
        return;
    }

    this->source = source;
    gst_object_ref(this->source);

    mIsActive = true;

    // Start
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_main_loop_run(this->gst_loop);

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    gst_object_unref(this->source);
    g_main_loop_unref(this->gst_loop);
    this->gst_loop = nullptr;
    this->source = nullptr;
}

void GstCameraPluginPrivate::stopGstThread() {
    if (this->gst_loop) {
        g_main_loop_quit(this->gst_loop);
    }
}

GstCameraPluginPrivate::GstCameraPluginPrivate()
    : width(0), height(0), depth(0) {}

GstCameraPluginPrivate::~GstCameraPluginPrivate() {
    if (this->gst_loop) {
        g_main_loop_quit(this->gst_loop);
    }
}

void GstCameraPluginPrivate::cbVideoStream(const gz::msgs::Int32 &_msg) {
    gzwarn << "Video Streaming callback: " << _msg.data() << "\n";
    int enable = _msg.data();
    if (enable)
        startStreaming();
    else
        stopStreaming();
}

void GstCameraPluginPrivate::startStreaming() {
    if (!mIsActive) {
        /* start the gstreamer event loop */
        pthread_create(&mThreadId, NULL, start_thread, this);
    }
}

void GstCameraPluginPrivate::stopStreaming() {
    if (mIsActive) {
        stopGstThread();
        pthread_join(mThreadId, NULL);
        mIsActive = false;
    }
}

void GstCameraPluginPrivate::OnImage(const msgs::Image &msg) {
    if (!mIsActive)
        return;

    // Alloc buffer
    const guint size = this->width * this->height * 1.5;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);

    if (!buffer) {
        gzerr << "gst_buffer_new_allocate failed" << endl;
        return;
    }

    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gzerr << "gst_buffer_map failed" << endl;
        return;
    }

    // Color Conversion from RGB to YUV
    Mat frame = Mat(this->height, this->width, CV_8UC3);
    Mat frameYUV = Mat(this->height, this->width, CV_8UC3);
    frame.data = (uchar *)msg.data().c_str();

    cvtColor(frame, frameYUV, COLOR_RGB2YUV_I420);
    memcpy(map.data, frameYUV.data, size);
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret =
        gst_app_src_push_buffer(GST_APP_SRC(this->source), buffer);

    if (ret != GST_FLOW_OK) {
        /* something wrong, stop pushing */
        gzerr << "gst_app_src_push_buffer failed" << endl;
        //g_main_loop_quit(this->gst_loop);
    }
}

}  // namespace gz
