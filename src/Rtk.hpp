/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_RTK_HPP_
#define JP_GNSS_HAT_RTK_HPP_

#include <string>
#include <vector>

#include <jimmypaputto/GnssHat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <yaml-cpp/yaml.h>

#include <jp_gnss_hat/msg/gnss_config.hpp>
#include <jp_gnss_hat/msg/rtk_corrections.hpp>

#ifdef JP_GNSS_NTRIP
#include <rtcm_msgs/msg/message.hpp>
#endif


namespace JimmyPaputto
{

class RtkBridge
{
public:
    RtkBridge(rclcpp_lifecycle::LifecycleNode& node, const std::string& frameId);
    ~RtkBridge();

    bool initialize(const GnssConfig& config, IRtk* rtk,
        [[maybe_unused]] bool useNtripRtcm,
        const std::string& correctionsTopic,
        [[maybe_unused]] const std::string& ntripTopic);
    void pollAndPublish();
    void onCorrections(const jp_gnss_hat::msg::RtkCorrections::SharedPtr msg);

    static RtkConfig fromYaml(const YAML::Node& rtkNode);
    static void toYaml(YAML::Emitter& out, const RtkConfig& cfg);

    static void toMsg(const RtkConfig& cfg, jp_gnss_hat::msg::GnssConfig& msg);
    static RtkConfig fromMsg(const jp_gnss_hat::msg::GnssConfig& msg);

private:
    rclcpp_lifecycle::LifecycleNode& node_;
    std::string frameId_;
    IBase* base_{nullptr};
    IRover* rover_{nullptr};
    rclcpp_lifecycle::LifecyclePublisher<
        jp_gnss_hat::msg::RtkCorrections>::SharedPtr pub_;
    rclcpp::Subscription<jp_gnss_hat::msg::RtkCorrections>::SharedPtr sub_;
    std::vector<std::vector<uint8_t>> bufor_;

#ifdef JP_GNSS_NTRIP
    rclcpp_lifecycle::LifecyclePublisher<
        rtcm_msgs::msg::Message>::SharedPtr stdRtcmPub_;
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr stdRtcmSub_;
    void onStandardRtcm(const rtcm_msgs::msg::Message::SharedPtr msg);
#endif
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_RTK_HPP_
