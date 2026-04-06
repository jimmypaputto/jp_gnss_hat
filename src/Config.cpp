/*
 * Jimmy Paputto 2026
 */

#include "Config.hpp"
#include "Rtk.hpp"

#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>


namespace JimmyPaputto
{

void Config::loadFromYaml(const std::string& path)
{
    YAML::Node yaml = YAML::LoadFile(path);

    nodeId_ = yaml["node_id"].as<std::string>("");
    publishStandardTopics_ = yaml["publish_standard_topics"].as<bool>(true);
    useNtripRtcm_ = yaml["use_ntrip_rtcm"].as<bool>(false);

    GnssConfig config{};

    config.measurementRate_Hz = yaml["measurement_rate_hz"].as<uint16_t>(1);
    config.saveToFlash = yaml["save_to_flash"].as<bool>(false);

    config.dynamicModel = static_cast<EDynamicModel>(
        yaml["dynamic_model"].as<uint8_t>(0));
    if (getDynamicModelName(config.dynamicModel) == "Unknown")
    {
        throw std::runtime_error(
            "Invalid dynamic_model: " +
            std::to_string(static_cast<int>(config.dynamicModel)));
    }

    const auto tpNode = yaml["timepulse"];
    config.timepulsePinConfig.active = tpNode["active"].as<bool>(true);
    config.timepulsePinConfig.fixedPulse.frequency =
        tpNode["fixed_pulse"]["frequency"].as<uint32_t>(1);
    config.timepulsePinConfig.fixedPulse.pulseWidth =
        tpNode["fixed_pulse"]["pulse_width"].as<float>(0.1f);

    if (tpNode["pulse_when_no_fix"])
    {
        TimepulsePinConfig::Pulse noFixPulse;
        noFixPulse.frequency =
            tpNode["pulse_when_no_fix"]["frequency"].as<uint32_t>(1);
        noFixPulse.pulseWidth =
            tpNode["pulse_when_no_fix"]["pulse_width"].as<float>(0.1f);
        config.timepulsePinConfig.pulseWhenNoFix = noFixPulse;
    }

    const auto polarityStr = tpNode["polarity"].as<std::string>("rising");
    config.timepulsePinConfig.polarity = (polarityStr == "falling")
        ? ETimepulsePinPolarity::FallingEdgeAtTopOfSecond
        : ETimepulsePinPolarity::RisingEdgeAtTopOfSecond;

    const auto geoNode = yaml["geofencing"];
    if (geoNode && geoNode.IsDefined())
    {
        GnssConfig::Geofencing geo{};
        geo.confidenceLevel = geoNode["confidence_level"].as<uint8_t>(0);

        if (geoNode["pio_pin_polarity"])
        {
            const auto polStr =
                geoNode["pio_pin_polarity"].as<std::string>("low_means_inside");
            geo.pioPinPolarity = (polStr == "low_means_outside")
                ? EPioPinPolarity::LowMeansOutside
                : EPioPinPolarity::LowMeansInside;
        }

        const auto fences = geoNode["geofences"];
        if (fences && fences.IsSequence())
        {
            for (size_t i = 0; i < std::min(fences.size(), size_t{4}); ++i)
            {
                Geofence gf;
                gf.lat    = fences[i]["lat"].as<float>();
                gf.lon    = fences[i]["lon"].as<float>();
                gf.radius = fences[i]["radius"].as<float>();
                geo.geofences.push_back(gf);
            }
        }

        config.geofencing = geo;
    }

    const auto rtkNode = yaml["rtk"];
    if (rtkNode && rtkNode.IsDefined())
        config.rtk = RtkBridge::fromYaml(rtkNode);

    gnssConfig_ = config;
}

void Config::saveToYaml(const std::string& path) const
{
    YAML::Emitter out;
    out << YAML::BeginMap;

    if (!nodeId_.empty())
        out << YAML::Key << "node_id" << YAML::Value << nodeId_;

    out << YAML::Key << "measurement_rate_hz"
        << YAML::Value << gnssConfig_.measurementRate_Hz;
    out << YAML::Key << "publish_standard_topics"
        << YAML::Value << publishStandardTopics_;
    out << YAML::Key << "use_ntrip_rtcm"
        << YAML::Value << useNtripRtcm_;
    out << YAML::Key << "save_to_flash"
        << YAML::Value << gnssConfig_.saveToFlash;
    out << YAML::Key << "dynamic_model"
        << YAML::Value << static_cast<int>(gnssConfig_.dynamicModel);

    out << YAML::Key << "timepulse" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "active"
        << YAML::Value << gnssConfig_.timepulsePinConfig.active;
    out << YAML::Key << "fixed_pulse" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "frequency"
        << YAML::Value << gnssConfig_.timepulsePinConfig.fixedPulse.frequency;
    out << YAML::Key << "pulse_width"
        << YAML::Value << gnssConfig_.timepulsePinConfig.fixedPulse.pulseWidth;
    out << YAML::EndMap;
    if (gnssConfig_.timepulsePinConfig.pulseWhenNoFix)
    {
        out << YAML::Key << "pulse_when_no_fix" << YAML::Value
            << YAML::BeginMap;
        out << YAML::Key << "frequency"
            << YAML::Value
            << gnssConfig_.timepulsePinConfig.pulseWhenNoFix->frequency;
        out << YAML::Key << "pulse_width"
            << YAML::Value
            << gnssConfig_.timepulsePinConfig.pulseWhenNoFix->pulseWidth;
        out << YAML::EndMap;
    }
    out << YAML::Key << "polarity" << YAML::Value
        << (gnssConfig_.timepulsePinConfig.polarity ==
                ETimepulsePinPolarity::FallingEdgeAtTopOfSecond
            ? "falling" : "rising");
    out << YAML::EndMap;

    if (gnssConfig_.geofencing)
    {
        out << YAML::Key << "geofencing" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "confidence_level"
            << YAML::Value
            << static_cast<int>(gnssConfig_.geofencing->confidenceLevel);
        if (gnssConfig_.geofencing->pioPinPolarity)
        {
            out << YAML::Key << "pio_pin_polarity" << YAML::Value
                << (*gnssConfig_.geofencing->pioPinPolarity ==
                        EPioPinPolarity::LowMeansOutside
                    ? "low_means_outside" : "low_means_inside");
        }
        out << YAML::Key << "geofences" << YAML::Value << YAML::BeginSeq;
        for (const auto& gf : gnssConfig_.geofencing->geofences)
        {
            out << YAML::BeginMap;
            out << YAML::Key << "lat" << YAML::Value << gf.lat;
            out << YAML::Key << "lon" << YAML::Value << gf.lon;
            out << YAML::Key << "radius" << YAML::Value << gf.radius;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
    }

    if (gnssConfig_.rtk)
        RtkBridge::toYaml(out, *gnssConfig_.rtk);

    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open())
    {
        throw std::runtime_error(
            "Cannot open config file for writing: " + path);
    }
    fout << out.c_str();
}

jp_gnss_hat::msg::GnssConfig Config::toMsg() const
{
    jp_gnss_hat::msg::GnssConfig msg;
    msg.measurement_rate_hz = gnssConfig_.measurementRate_Hz;
    msg.dynamic_model = static_cast<uint8_t>(gnssConfig_.dynamicModel);
    msg.publish_standard_topics = publishStandardTopics_;
    msg.use_ntrip_rtcm = useNtripRtcm_;
    msg.save_to_flash = gnssConfig_.saveToFlash;

    const auto& tp = gnssConfig_.timepulsePinConfig;
    msg.timepulse_active = tp.active;
    msg.timepulse_frequency = tp.fixedPulse.frequency;
    msg.timepulse_pulse_width = tp.fixedPulse.pulseWidth;
    msg.timepulse_has_no_fix_pulse = tp.pulseWhenNoFix.has_value();
    if (tp.pulseWhenNoFix)
    {
        msg.timepulse_no_fix_frequency = tp.pulseWhenNoFix->frequency;
        msg.timepulse_no_fix_pulse_width = tp.pulseWhenNoFix->pulseWidth;
    }
    msg.timepulse_polarity = static_cast<uint8_t>(tp.polarity);

    msg.geofencing_enabled = gnssConfig_.geofencing.has_value();
    if (gnssConfig_.geofencing)
    {
        msg.geofencing_confidence_level =
            gnssConfig_.geofencing->confidenceLevel;
        msg.geofencing_has_pio_pin_polarity =
            gnssConfig_.geofencing->pioPinPolarity.has_value();
        if (gnssConfig_.geofencing->pioPinPolarity)
            msg.geofencing_pio_pin_polarity =
                static_cast<uint8_t>(*gnssConfig_.geofencing->pioPinPolarity);
        for (const auto& gf : gnssConfig_.geofencing->geofences)
        {
            jp_gnss_hat::msg::Geofence fence;
            fence.lat = gf.lat;
            fence.lon = gf.lon;
            fence.radius = gf.radius;
            msg.geofences.push_back(fence);
        }
    }

    msg.rtk_enabled = gnssConfig_.rtk.has_value();
    if (gnssConfig_.rtk)
        RtkBridge::toMsg(*gnssConfig_.rtk, msg);

    return msg;
}

void Config::fromMsg(const jp_gnss_hat::msg::GnssConfig& msg)
{
    GnssConfig config{};
    config.measurementRate_Hz = msg.measurement_rate_hz;
    config.dynamicModel = static_cast<EDynamicModel>(msg.dynamic_model);
    config.saveToFlash = msg.save_to_flash;

    config.timepulsePinConfig.active = msg.timepulse_active;
    config.timepulsePinConfig.fixedPulse.frequency = msg.timepulse_frequency;
    config.timepulsePinConfig.fixedPulse.pulseWidth = msg.timepulse_pulse_width;
    if (msg.timepulse_has_no_fix_pulse)
    {
        TimepulsePinConfig::Pulse noFix;
        noFix.frequency = msg.timepulse_no_fix_frequency;
        noFix.pulseWidth = msg.timepulse_no_fix_pulse_width;
        config.timepulsePinConfig.pulseWhenNoFix = noFix;
    }
    config.timepulsePinConfig.polarity =
        static_cast<ETimepulsePinPolarity>(msg.timepulse_polarity);

    if (msg.geofencing_enabled)
    {
        GnssConfig::Geofencing geo{};
        geo.confidenceLevel = msg.geofencing_confidence_level;
        if (msg.geofencing_has_pio_pin_polarity)
            geo.pioPinPolarity =
                static_cast<EPioPinPolarity>(msg.geofencing_pio_pin_polarity);
        for (const auto& f : msg.geofences)
        {
            Geofence gf;
            gf.lat = f.lat;
            gf.lon = f.lon;
            gf.radius = f.radius;
            geo.geofences.push_back(gf);
        }
        config.geofencing = geo;
    }

    if (msg.rtk_enabled)
        config.rtk = RtkBridge::fromMsg(msg);

    publishStandardTopics_ = msg.publish_standard_topics;
    useNtripRtcm_ = msg.use_ntrip_rtcm;
    gnssConfig_ = config;
}

jp_gnss_hat::srv::GetGeofencingCfg::Response Config::toGeofencingResponse() const
{
    jp_gnss_hat::srv::GetGeofencingCfg::Response response{};

    const auto& geo = gnssConfig_.geofencing;
    if (!geo)
    {
        response.enabled = false;
        return response;
    }

    response.enabled = true;
    response.pio_enabled = geo->pioPinPolarity.has_value();
    if (geo->pioPinPolarity)
        response.pin_polarity = static_cast<uint8_t>(*geo->pioPinPolarity);
    response.confidence_level = geo->confidenceLevel;
    for (const auto& gf : geo->geofences)
    {
        jp_gnss_hat::msg::Geofence fence;
        fence.lat = gf.lat;
        fence.lon = gf.lon;
        fence.radius = gf.radius;
        response.geofences.push_back(fence);
    }

    return response;
}

std::string Config::getDynamicModelName(const EDynamicModel model)
{
    switch (model)
    {
        case EDynamicModel::Portable:    return "Portable";
        case EDynamicModel::Stationary:  return "Stationary";
        case EDynamicModel::Pedestrian:  return "Pedestrian";
        case EDynamicModel::Automotive:  return "Automotive";
        case EDynamicModel::Sea:         return "Sea";
        case EDynamicModel::Airborne1G:  return "Airborne1G";
        case EDynamicModel::Airborne2G:  return "Airborne2G";
        case EDynamicModel::Airborne4G:  return "Airborne4G";
        case EDynamicModel::Wrist:       return "Wrist";
        case EDynamicModel::Bike:        return "Bike";
        case EDynamicModel::Mower:       return "Mower";
        case EDynamicModel::Escooter:    return "Escooter";
        default:                         return "Unknown";
    }
}

std::string Config::readNodeId(const std::string& path)
{
    try
    {
        auto yaml = YAML::LoadFile(path);
        return yaml["node_id"].as<std::string>("");
    }
    catch (...)
    {
        return "";
    }
}

}  // JimmyPaputto
