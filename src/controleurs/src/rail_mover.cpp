/**
 * rail_mover.cpp
 * ==============
 * Déplace le robot (probo11) et le curseur (rail_curseur) sur le rail
 * via ignition::transport directement — sans subprocess, sans latence.
 *
 * À 100 Hz, entre deux appels set_pose : chute gravité = 0.5×9.81×(0.01)² ≈ 0.5 mm.
 *
 * Topics ROS 2
 * ------------
 *   SUB  /target_rail_pos          std_msgs/Float64   position cible (m)
 *   PUB  /rail_mover/done          std_msgs/Bool      True = arrivée
 *   PUB  /current_rail_position    std_msgs/Float64   position interpolée
 *
 * Paramètres
 * ----------
 *   initial_rail_position  double  défaut 0.0    position de départ (m)
 *   rail_speed             double  défaut 0.35   vitesse (m/s)
 *   world_name             string  défaut "empty" nom du world Gazebo
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>

#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

// ── Géométrie ────────────────────────────────────────────────────────────────
static constexpr double Y_BASE  = -0.85;   // y robot quand rail_position = 0
static constexpr double Z_ARM   =  0.0;    // z origine probo11
static constexpr double Z_CURS  =  1.03;   // z curseur
static constexpr double GOAL_TOL = 1e-3;   // seuil d'arrivée (m)

// ════════════════════════════════════════════════════════════════════════════
class RailMover : public rclcpp::Node
{
public:
    RailMover() : Node("rail_mover"), moving_(false)
    {
        // ── Paramètres ───────────────────────────────────────────────────
        this->declare_parameter("initial_rail_position", 0.0);
        this->declare_parameter("rail_speed", 0.35);
        this->declare_parameter("world_name", std::string("empty"));

        rail_pos_   = this->get_parameter("initial_rail_position")
                          .get_parameter_value().get<double>();
        rail_speed_ = this->get_parameter("rail_speed")
                          .get_parameter_value().get<double>();
        world_name_ = this->get_parameter("world_name")
                          .get_parameter_value().get<std::string>();

        target_pos_ = rail_pos_;
        service_name_ = "/world/" + world_name_ + "/set_pose";

        // ── Publishers ───────────────────────────────────────────────────
        pub_done_ = create_publisher<std_msgs::msg::Bool>(
            "/rail_mover/done", 10);
        pub_pos_  = create_publisher<std_msgs::msg::Float64>(
            "/current_rail_position", 10);

        // ── Subscriber ───────────────────────────────────────────────────
        sub_target_ = create_subscription<std_msgs::msg::Float64>(
            "/target_rail_pos", 5,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                onTarget(msg->data);
            });

        // ── Timer de contrôle à 100 Hz ───────────────────────────────────
        timer_ = create_wall_timer(10ms,
            [this]() { controlLoop(); });

        publishCurrent();
        RCLCPP_INFO(get_logger(),
            "rail_mover prêt — pos=%.3f m  vitesse=%.2f m/s  service=%s",
            rail_pos_, rail_speed_, service_name_.c_str());
    }

private:
    // ── Callback cible ───────────────────────────────────────────────────────
    void onTarget(double new_pos)
    {
        new_pos = std::max(0.0, std::min(1.7, new_pos));

        if (std::abs(new_pos - rail_pos_) < GOAL_TOL) {
            RCLCPP_INFO(get_logger(), "Rail déjà en position.");
            publishDone(true);
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Rail : %.3f → %.3f m  (v=%.2f m/s)",
            rail_pos_, new_pos, rail_speed_);

        target_pos_ = new_pos;
        moving_     = true;
    }

    // ── Boucle de contrôle (100 Hz) ──────────────────────────────────────────
    void controlLoop()
    {
        // Même à l'arrêt : on maintient la position pour contrecarrer la gravité.
        if (!moving_) {
            setPoseBoth(Y_BASE + rail_pos_);
            return;
        }

        double remaining = target_pos_ - rail_pos_;

        if (std::abs(remaining) <= GOAL_TOL) {
            // Arrivée — appel final bloquant pour confirmer
            rail_pos_ = target_pos_;
            setPoseBoth(Y_BASE + rail_pos_);
            publishCurrent();
            moving_ = false;
            RCLCPP_INFO(get_logger(), "Rail en position : %.3f m", rail_pos_);
            publishDone(true);
            return;
        }

        double step = rail_speed_ * 0.01;   // v × dt (dt = 10 ms)
        if (std::abs(remaining) <= step)
            rail_pos_ = target_pos_;
        else
            rail_pos_ += step * (remaining > 0.0 ? 1.0 : -1.0);

        setPoseBoth(Y_BASE + rail_pos_);
        publishCurrent();
    }

    // ── set_pose via ignition transport (non bloquant côté timer) ────────────
    void setPoseBoth(double y)
    {
        // Les deux appels sont faits en séquence — chacun <5 ms via ign transport.
        setPose("probo11",      y, Z_ARM);
        setPose("rail_curseur", y, Z_CURS);
    }

    bool setPose(const std::string & model_name, double y, double z)
    {
        ignition::msgs::Pose req;
        req.set_name(model_name);
        req.mutable_position()->set_x(0.0);
        req.mutable_position()->set_y(y);
        req.mutable_position()->set_z(z);
        // Orientation identité (quaternion w=1)
        req.mutable_orientation()->set_w(1.0);
        req.mutable_orientation()->set_x(0.0);
        req.mutable_orientation()->set_y(0.0);
        req.mutable_orientation()->set_z(0.0);

        ignition::msgs::Boolean rep;
        bool result = false;
        // Timeout 200 ms — bien suffisant pour un appel local
        bool ok = ign_node_.Request(service_name_, req, 200, rep, result);
        if (!ok || !result) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "set_pose(%s) échec — Gazebo prêt ?", model_name.c_str());
        }
        return ok && result;
    }

    // ── Helpers ─────────────────────────────────────────────────────────────
    void publishDone(bool v)
    {
        std_msgs::msg::Bool msg;
        msg.data = v;
        pub_done_->publish(msg);
    }

    void publishCurrent()
    {
        std_msgs::msg::Float64 msg;
        msg.data = rail_pos_;
        pub_pos_->publish(msg);
    }

    // ── Membres ──────────────────────────────────────────────────────────────
    ignition::transport::Node ign_node_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_done_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_;
    rclcpp::TimerBase::SharedPtr timer_;

    double      rail_pos_;
    double      target_pos_;
    double      rail_speed_;
    bool        moving_;
    std::string world_name_;
    std::string service_name_;
};

// ════════════════════════════════════════════════════════════════════════════
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RailMover>());
    rclcpp::shutdown();
    return 0;
}
