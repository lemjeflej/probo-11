/**
 * cartesian_commander.cpp
 * =======================
 * Contrôle cartésien du FR3 sans MoveIt.
 *
 * Principe
 * --------
 *   1. Reçoit une pose cible sur /target_pose (geometry_msgs/PoseStamped)
 *   2. Lit la configuration articulaire courante depuis /joint_states
 *   3. Calcule la cinématique inverse (IK) via KDL  (NR + pseudoinverse)
 *   4. Envoie la trajectoire à fr3_arm_controller via l'action
 *      FollowJointTrajectory
 *
 * Topics/Actions
 * --------------
 *   SUB  /target_pose          geometry_msgs/PoseStamped
 *   SUB  /joint_states         sensor_msgs/JointState
 *   ACT  /fr3_arm_controller/follow_joint_trajectory
 *        control_msgs/FollowJointTrajectory
 *
 * Paramètres ROS 2
 * ----------------
 *   move_duration   (double, défaut 3.0)  durée du mouvement en secondes
 *   ik_max_iter     (int,    défaut 200)  itérations max du solveur IK
 *   ik_tolerance    (double, défaut 1e-5) tolérance de convergence (m)
 *
 * Chaîne cinématique KDL
 * ----------------------
 *   fr3_link0 → sonde_tcp  (7 joints rotatifs + 2 joints fixes)
 *   L'IK ne porte que sur les 7 joints rotatifs.
 *
 * Orientation d'entrée
 * --------------------
 *   Le champ pose.orientation de PoseStamped est un quaternion (x y z w).
 *   Le frame de référence est world (ou le frame de base du robot).
 */

#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>


using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;

// Noms des joints du bras dans l'ordre KDL / URDF
static const std::vector<std::string> ARM_JOINTS = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
};

// Limites articulaires FR3 (rad) — issues de franka_description
// Ordre : joint1 .. joint7
static const double Q_MIN[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
static const double Q_MAX[7] = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

// Configuration "ready" — pose initiale au démarrage
// q2=-45°, q4=-135°, q6=90°, q7=45° (même que les initial_value URDF)
static const double Q_READY[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

// Configuration "home" utilisée comme seed initial pour l'IK
static const double Q_HOME[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};


class CartesianCommander : public rclcpp::Node
{
public:
    CartesianCommander() : Node("cartesian_commander"), kdl_ready_(false)
    {
        // ── Paramètres ──────────────────────────────────────────────────
        move_duration_ = this->declare_parameter<double>("move_duration", 3.0);
        ik_max_iter_   = this->declare_parameter<int>("ik_max_iter", 200);
        ik_tolerance_  = this->declare_parameter<double>("ik_tolerance", 1e-5);

        // ── Abonnements ─────────────────────────────────────────────────
        sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianCommander::onJointState, this, std::placeholders::_1));

        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 5,
            std::bind(&CartesianCommander::onTargetPose, this, std::placeholders::_1));

        // ── Client d'action ─────────────────────────────────────────────
        action_client_ = rclcpp_action::create_client<FollowJT>(
            this, "/fr3_arm_controller/follow_joint_trajectory");

        // ── Publisher busy → GUI désactive le bouton pendant le mouvement ─
        pub_busy_ = this->create_publisher<std_msgs::msg::Bool>(
            "/cartesian_commander/busy", 10);

        // ── Initialisation KDL (asynchrone, attend robot_state_publisher) ─
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CartesianCommander::tryInitKDL, this));

        publishBusy(false);  // démarrage : pas occupé

        RCLCPP_INFO(get_logger(),
            "cartesian_commander demarré.\n"
            "  Attente de robot_state_publisher pour construire la chaine KDL...\n"
            "  Envoyez une pose sur /target_pose pour commander le robot.");
    }

private:
    // ====================================================================
    // Initialisation KDL
    // ====================================================================

    void tryInitKDL()
    {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(
            this, "robot_state_publisher");

        if (!client->wait_for_service(std::chrono::milliseconds(500))) {
            return;  // pas encore disponible, réessai au prochain tick (1s)
        }

        auto params = client->get_parameters({"robot_description"});
        if (params.empty() ||
            params[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
            return;
        }

        std::string urdf = params[0].as_string();

        // Parser l'URDF en arbre KDL
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(urdf, tree)) {
            RCLCPP_ERROR(get_logger(), "Echec du parsing URDF vers KDL.");
            return;
        }

        // Extraire fr3_link0 → sonde_tcp
        // 7 joints rotatifs + 2 joints fixes (sonde_joint + sonde_tcp_joint)
        // KDL::getNrOfJoints() ne compte que les joints non-fixes → retourne 7
        if (!tree.getChain("fr3_link0", "sonde_tcp", chain_)) {
            RCLCPP_ERROR(get_logger(),
                "Impossible d'extraire fr3_link0 -> sonde_tcp. "
                "Vérifier que srr.xacro est bien chargé.");
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Chaine KDL : %d segments, %d joints mobiles.",
            chain_.getNrOfSegments(), chain_.getNrOfJoints());

        // Solveur FK
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // Solveurs IK
        int nj = static_cast<int>(chain_.getNrOfJoints());  // = 7
        KDL::JntArray q_min(nj), q_max(nj);
        for (int i = 0; i < nj; ++i) {
            q_min(i) = Q_MIN[i];
            q_max(i) = Q_MAX[i];
        }

        auto ik_vel = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_  = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
            chain_,
            q_min, q_max,
            *fk_solver_,
            *ik_vel,
            ik_max_iter_,
            ik_tolerance_);

        kdl_ready_ = true;
        init_timer_->cancel();

        RCLCPP_INFO(get_logger(), "KDL pret. Envoi de la trajectoire vers la pose ready...");
        goToReady();
    }

    // ====================================================================
    // Trajectoire vers la pose "ready" au démarrage
    // ====================================================================

    void goToReady()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(),
                "Action server fr3_arm_controller non disponible pour goToReady.");
            return;
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = ARM_JOINTS;

        // Point de départ : configuration courante (joints à 0 au spawn)
        trajectory_msgs::msg::JointTrajectoryPoint pt_start;
        pt_start.positions.assign(current_q_, current_q_ + 7);
        pt_start.velocities.resize(7, 0.0);
        pt_start.accelerations.resize(7, 0.0);
        pt_start.time_from_start = rclcpp::Duration::from_seconds(0.0);

        // Point d'arrivée : pose ready
        trajectory_msgs::msg::JointTrajectoryPoint pt_ready;
        pt_ready.positions.assign(Q_READY, Q_READY + 7);
        pt_ready.velocities.resize(7, 0.0);
        pt_ready.accelerations.resize(7, 0.0);
        pt_ready.time_from_start = rclcpp::Duration::from_seconds(move_duration_);

        traj.points = {pt_start, pt_ready};

        auto goal = FollowJT::Goal();
        goal.trajectory          = traj;
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);

        auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
        opts.result_callback =
            [this](const GoalHandleFollowJT::WrappedResult & res) {
                if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(),
                        "Pose ready atteinte. Publiez sur /target_pose pour commander le robot.");
                } else {
                    RCLCPP_ERROR(get_logger(), "Echec du mouvement vers ready.");
                }
            };

        action_client_->async_send_goal(goal, opts);
    }

    // ====================================================================
    // Helper : publie l'état occupé/libre sur /cartesian_commander/busy
    // ====================================================================

    void publishBusy(bool busy)
    {
        std_msgs::msg::Bool msg;
        msg.data = busy;
        pub_busy_->publish(msg);
    }

    // ====================================================================
    // Callback : mise à jour de la configuration courante
    // ====================================================================

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < ARM_JOINTS.size(); ++j) {
                if (msg->name[i] == ARM_JOINTS[j]) {
                    current_q_[j] = msg->position[i];
                }
            }
        }
    }

    // ====================================================================
    // Callback : réception d'une pose cible
    // ====================================================================

    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!kdl_ready_) {
            RCLCPP_WARN(get_logger(),
                "Pose reçue mais KDL non initialisé. Ignorer.");
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(),
                "Action server fr3_arm_controller non disponible.");
            return;
        }

        // ── Frame KDL cible ─────────────────────────────────────────────
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;

        KDL::Frame target;
        target.p = KDL::Vector(p.x, p.y, p.z);
        target.M = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);

        RCLCPP_INFO(get_logger(),
            "Pose cible : pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
            p.x, p.y, p.z, q.x, q.y, q.z, q.w);

        // ── IK — seed : config courante ──────────────────────────────────
        int nj = static_cast<int>(chain_.getNrOfJoints());
        KDL::JntArray q_seed(nj), q_out(nj);
        for (int i = 0; i < nj; ++i) {
            q_seed(i) = current_q_[i];
        }

        int ret = ik_solver_->CartToJnt(q_seed, target, q_out);

        if (ret < 0) {
            RCLCPP_WARN(get_logger(),
                "IK echouée depuis config courante (code %d). "
                "Tentative depuis home...", ret);

            for (int i = 0; i < nj; ++i) {
                q_seed(i) = Q_HOME[i];
            }
            ret = ik_solver_->CartToJnt(q_seed, target, q_out);

            if (ret < 0) {
                RCLCPP_ERROR(get_logger(),
                    "IK impossible (code %d). Pose hors espace de travail.", ret);
                return;
            }
        }

        // ── Vérification FK ─────────────────────────────────────────────
        KDL::Frame fk_check;
        fk_solver_->JntToCart(q_out, fk_check);
        double pos_err = (fk_check.p - target.p).Norm();
        RCLCPP_INFO(get_logger(),
            "IK reussie. Erreur residuelle : %.6f m", pos_err);
        if (pos_err > 0.005) {
            RCLCPP_WARN(get_logger(),
                "Erreur IK > 5 mm — pose probablement hors espace de travail.");
        }

        // ── Trajectoire ─────────────────────────────────────────────────
        // Deux points :  t=0 (départ, vitesse nulle)  /  t=T (cible, vitesse nulle)
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = ARM_JOINTS;

        trajectory_msgs::msg::JointTrajectoryPoint pt_start;
        pt_start.positions.resize(7);
        pt_start.velocities.resize(7, 0.0);
        pt_start.accelerations.resize(7, 0.0);
        for (int i = 0; i < 7; ++i) {
            pt_start.positions[i] = current_q_[i];
        }
        pt_start.time_from_start = rclcpp::Duration::from_seconds(0.0);

        trajectory_msgs::msg::JointTrajectoryPoint pt_end;
        pt_end.positions.resize(7);
        pt_end.velocities.resize(7, 0.0);
        pt_end.accelerations.resize(7, 0.0);
        for (int i = 0; i < nj; ++i) {
            pt_end.positions[i] = q_out(i);
        }
        pt_end.time_from_start = rclcpp::Duration::from_seconds(move_duration_);

        traj.points = {pt_start, pt_end};

        // ── Envoi du goal ────────────────────────────────────────────────
        publishBusy(true);  // GUI : désactiver le bouton

        auto goal = FollowJT::Goal();
        goal.trajectory          = traj;
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);

        auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleFollowJT::SharedPtr & handle) {
                if (!handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejeté par le contrôleur.");
                    publishBusy(false);
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepté — mouvement en cours.");
                }
            };

        opts.result_callback =
            [this](const GoalHandleFollowJT::WrappedResult & res) {
                switch (res.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(),  "Mouvement terminé.");  break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "Mouvement aborté.");   break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(get_logger(),  "Mouvement annulé.");   break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Résultat inconnu.");
                }
                publishBusy(false);  // GUI : réactiver le bouton
            };

        action_client_->async_send_goal(goal, opts);
        RCLCPP_INFO(get_logger(),
            "Trajectoire envoyée (durée : %.1f s).", move_duration_);
    }

    // ====================================================================
    // Membres
    // ====================================================================

    bool kdl_ready_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive>  fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_solver_;

    double current_q_[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

    double move_duration_;
    int    ik_max_iter_;
    double ik_tolerance_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    sub_joints_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pub_busy_;
    rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianCommander>());
    rclcpp::shutdown();
    return 0;
}
