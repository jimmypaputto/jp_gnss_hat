/*
 * Jimmy Paputto 2026
 */

#include "Rtk.hpp"

#include <yaml-cpp/yaml.h>


namespace JimmyPaputto
{

RtkBridge::RtkBridge(
    rclcpp_lifecycle::LifecycleNode& node, const std::string& frameId)
:   node_(node),
    frameId_(frameId)
{
}

RtkBridge::~RtkBridge()
{
    base_ = nullptr;
    rover_ = nullptr;
    sub_.reset();
    pub_.reset();
#ifdef JP_GNSS_NTRIP
    stdRtcmSub_.reset();
    stdRtcmPub_.reset();
#endif
}

bool RtkBridge::initialize(const GnssConfig& config, IRtk* rtk,
    [[maybe_unused]] bool useNtripRtcm,
    const std::string& correctionsTopic,
    [[maybe_unused]] const std::string& ntripTopic)
{
    if (!rtk)
    {
        RCLCPP_ERROR(node_.get_logger(),
            "Failed to initialize, RTK pointing to null");
        return false;
    }

    if (config.rtk->mode == ERtkMode::Base && rtk->base())
    {
        base_ = rtk->base();
        pub_ = node_.create_publisher<jp_gnss_hat::msg::RtkCorrections>(
            correctionsTopic, 10);
        RCLCPP_INFO(node_.get_logger(),
            "RTK Base mode - publishing corrections");

#ifdef JP_GNSS_NTRIP
        if (useNtripRtcm)
        {
            stdRtcmPub_ = node_.create_publisher<rtcm_msgs::msg::Message>(
                ntripTopic, 10);
            RCLCPP_INFO(node_.get_logger(),
                "NTRIP RTCM bridge enabled - publishing on %s",
                ntripTopic.c_str());
        }
#endif
    }
    else if (config.rtk->mode == ERtkMode::Rover && rtk->rover())
    {
        rover_ = rtk->rover();
        sub_ = node_.create_subscription<jp_gnss_hat::msg::RtkCorrections>(
            correctionsTopic, 10,
            std::bind(&RtkBridge::onCorrections, this, std::placeholders::_1));
        RCLCPP_INFO(node_.get_logger(),
            "RTK Rover mode - subscribing to corrections");

#ifdef JP_GNSS_NTRIP
        if (useNtripRtcm)
        {
            stdRtcmSub_ = node_.create_subscription<rtcm_msgs::msg::Message>(
                ntripTopic, 10,
                std::bind(&RtkBridge::onStandardRtcm, this,
                    std::placeholders::_1));
            RCLCPP_INFO(node_.get_logger(),
                "NTRIP RTCM bridge enabled - subscribing on %s",
                ntripTopic.c_str());
        }
#endif
    }

    return true;
}

void RtkBridge::pollAndPublish()
{
    if (!base_)
        return;

    const auto frames = base_->getFullCorrections();

    jp_gnss_hat::msg::RtkCorrections msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = frameId_;

    for (const auto& frame : frames)
    {
        jp_gnss_hat::msg::Rtcm3Frame rtcm3Frame;
        rtcm3Frame.data.assign(frame.begin(), frame.end());
        msg.frames.push_back(std::move(rtcm3Frame));
    }

    pub_->publish(msg);

#ifdef JP_GNSS_NTRIP
    if (stdRtcmPub_)
    {
        for (const auto& frame : frames)
        {
            rtcm_msgs::msg::Message stdMsg;
            stdMsg.header.stamp = msg.header.stamp;
            stdMsg.header.frame_id = frameId_;
            stdMsg.message.assign(frame.begin(), frame.end());
            stdRtcmPub_->publish(stdMsg);
        }
    }
#endif
}

void RtkBridge::onCorrections(
    const jp_gnss_hat::msg::RtkCorrections::SharedPtr msg)
{
    if (!rover_)
        return;

    const size_t frameCount = msg->frames.size();
    bufor_.resize(frameCount);

    for (size_t i = 0; i < frameCount; ++i)
        bufor_[i] = msg->frames[i].data;

    rover_->applyCorrections(bufor_);
}

RtkConfig RtkBridge::fromYaml(const YAML::Node& rtkNode)
{
    RtkConfig cfg{};
    const auto modeStr = rtkNode["mode"].as<std::string>("rover");
    cfg.mode = (modeStr == "base") ? ERtkMode::Base : ERtkMode::Rover;

    if (cfg.mode == ERtkMode::Base && rtkNode["base"])
    {
        const auto baseNode = rtkNode["base"];
        const auto baseType = baseNode["type"].as<std::string>("survey_in");

        if (baseType == "survey_in")
        {
            BaseConfig::SurveyIn si;
            si.minimumObservationTime_s =
                baseNode["min_observation_time_s"].as<uint32_t>(60);
            si.requiredPositionAccuracy_m =
                baseNode["required_accuracy_m"].as<double>(2.0);
            cfg.base = BaseConfig{.mode = si};
        }
        else if (baseType == "fixed_ecef")
        {
            BaseConfig::FixedPosition fp;
            BaseConfig::FixedPosition::Ecef ecef;
            ecef.x_m = baseNode["x_m"].as<double>();
            ecef.y_m = baseNode["y_m"].as<double>();
            ecef.z_m = baseNode["z_m"].as<double>();
            fp.position = ecef;
            fp.positionAccuracy_m =
                baseNode["position_accuracy_m"].as<double>(0.01);
            cfg.base = BaseConfig{.mode = fp};
        }
        else if (baseType == "fixed_lla")
        {
            BaseConfig::FixedPosition fp;
            BaseConfig::FixedPosition::Lla lla;
            lla.latitude_deg = baseNode["latitude_deg"].as<double>();
            lla.longitude_deg = baseNode["longitude_deg"].as<double>();
            lla.height_m = baseNode["height_m"].as<double>();
            fp.position = lla;
            fp.positionAccuracy_m =
                baseNode["position_accuracy_m"].as<double>(0.01);
            cfg.base = BaseConfig{.mode = fp};
        }
    }

    return cfg;
}

void RtkBridge::toYaml(YAML::Emitter& out, const RtkConfig& cfg)
{
    out << YAML::Key << "rtk" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "mode" << YAML::Value
        << (cfg.mode == ERtkMode::Base ? "base" : "rover");

    if (cfg.base)
    {
        out << YAML::Key << "base" << YAML::Value << YAML::BeginMap;
        if (std::holds_alternative<BaseConfig::SurveyIn>(cfg.base->mode))
        {
            const auto& si =
                std::get<BaseConfig::SurveyIn>(cfg.base->mode);
            out << YAML::Key << "type" << YAML::Value << "survey_in";
            out << YAML::Key << "min_observation_time_s"
                << YAML::Value << si.minimumObservationTime_s;
            out << YAML::Key << "required_accuracy_m"
                << YAML::Value << si.requiredPositionAccuracy_m;
        }
        else
        {
            const auto& fp =
                std::get<BaseConfig::FixedPosition>(cfg.base->mode);
            if (std::holds_alternative<BaseConfig::FixedPosition::Ecef>(
                    fp.position))
            {
                const auto& ecef =
                    std::get<BaseConfig::FixedPosition::Ecef>(fp.position);
                out << YAML::Key << "type" << YAML::Value << "fixed_ecef";
                out << YAML::Key << "x_m" << YAML::Value << ecef.x_m;
                out << YAML::Key << "y_m" << YAML::Value << ecef.y_m;
                out << YAML::Key << "z_m" << YAML::Value << ecef.z_m;
            }
            else
            {
                const auto& lla =
                    std::get<BaseConfig::FixedPosition::Lla>(fp.position);
                out << YAML::Key << "type" << YAML::Value << "fixed_lla";
                out << YAML::Key << "latitude_deg"
                    << YAML::Value << lla.latitude_deg;
                out << YAML::Key << "longitude_deg"
                    << YAML::Value << lla.longitude_deg;
                out << YAML::Key << "height_m"
                    << YAML::Value << lla.height_m;
            }
            out << YAML::Key << "position_accuracy_m"
                << YAML::Value << fp.positionAccuracy_m;
        }
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
}

void RtkBridge::toMsg(const RtkConfig& cfg, jp_gnss_hat::msg::GnssConfig& msg)
{
    msg.rtk_mode = static_cast<uint8_t>(cfg.mode);
    if (cfg.base)
    {
        const auto& baseMode = cfg.base->mode;
        if (std::holds_alternative<BaseConfig::SurveyIn>(baseMode))
        {
            const auto& si = std::get<BaseConfig::SurveyIn>(baseMode);
            msg.rtk_base_type = 0;
            msg.rtk_min_observation_time_s = si.minimumObservationTime_s;
            msg.rtk_required_accuracy_m = si.requiredPositionAccuracy_m;
        }
        else
        {
            const auto& fp =
                std::get<BaseConfig::FixedPosition>(baseMode);
            msg.rtk_position_accuracy_m = fp.positionAccuracy_m;
            if (std::holds_alternative<BaseConfig::FixedPosition::Ecef>(
                    fp.position))
            {
                const auto& ecef =
                    std::get<BaseConfig::FixedPosition::Ecef>(fp.position);
                msg.rtk_base_type = 1;
                msg.rtk_ecef_x_m = ecef.x_m;
                msg.rtk_ecef_y_m = ecef.y_m;
                msg.rtk_ecef_z_m = ecef.z_m;
            }
            else
            {
                const auto& lla =
                    std::get<BaseConfig::FixedPosition::Lla>(fp.position);
                msg.rtk_base_type = 2;
                msg.rtk_lla_latitude_deg = lla.latitude_deg;
                msg.rtk_lla_longitude_deg = lla.longitude_deg;
                msg.rtk_lla_height_m = lla.height_m;
            }
        }
    }
}

RtkConfig RtkBridge::fromMsg(const jp_gnss_hat::msg::GnssConfig& msg)
{
    RtkConfig cfg{};
    cfg.mode = static_cast<ERtkMode>(msg.rtk_mode);

    if (cfg.mode != ERtkMode::Base)
        return cfg;

    if (msg.rtk_base_type == 0)
    {
        BaseConfig::SurveyIn si;
        si.minimumObservationTime_s = msg.rtk_min_observation_time_s;
        si.requiredPositionAccuracy_m = msg.rtk_required_accuracy_m;
        cfg.base = BaseConfig{.mode = si};
    }
    else if (msg.rtk_base_type == 1)
    {
        BaseConfig::FixedPosition fp;
        BaseConfig::FixedPosition::Ecef ecef;
        ecef.x_m = msg.rtk_ecef_x_m;
        ecef.y_m = msg.rtk_ecef_y_m;
        ecef.z_m = msg.rtk_ecef_z_m;
        fp.position = ecef;
        fp.positionAccuracy_m = msg.rtk_position_accuracy_m;
        cfg.base = BaseConfig{.mode = fp};
    }
    else if (msg.rtk_base_type == 2)
    {
        BaseConfig::FixedPosition fp;
        BaseConfig::FixedPosition::Lla lla;
        lla.latitude_deg = msg.rtk_lla_latitude_deg;
        lla.longitude_deg = msg.rtk_lla_longitude_deg;
        lla.height_m = msg.rtk_lla_height_m;
        fp.position = lla;
        fp.positionAccuracy_m = msg.rtk_position_accuracy_m;
        cfg.base = BaseConfig{.mode = fp};
    }

    return cfg;
}

#ifdef JP_GNSS_NTRIP
void RtkBridge::onStandardRtcm(const rtcm_msgs::msg::Message::SharedPtr msg)
{
    if (!rover_)
        return;

    bufor_.resize(1);
    bufor_[0] = msg->message;
    rover_->applyCorrections(bufor_);
}
#endif

}  // JimmyPaputto
