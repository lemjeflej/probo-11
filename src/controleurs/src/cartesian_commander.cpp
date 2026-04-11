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
        move_duration_  = this->declare_parameter<double>("move_duration", 3.0);
        ik_max_iter_    = this->declare_parameter<int>("ik_max_iter", 200);
        ik_tolerance_   = this->declare_parameter<double>("ik_tolerance", 1e-5);
        // Position du rail (m) — fixée au lancement, identique à rail_position xacro.
        // fr3_link0 est à y = -0.85 + rail_position dans world.
        // La chaîne KDL part de fr3_link0, donc aucun offset y n'est nécessaire
        // dans onTargetPose : les poses GUI sont déjà en frame fr3_link0.
        // Ce paramètre est conservé pour info / affichage et usage futur.
        rail_position_  = this->declare_parameter<double>("rail_position", 0.0);

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

        // ── Publisher pose TCP courante → GUI affiche la position du robot ─
        pub_tcp_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/current_tcp_pose", 10);

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
        // SyncParametersClient crée un executor interne et y ajoute le node.
        // Si on lui passe `this`, il entre en conflit avec rclcpp::spin.
        // Solution : node temporaire dédié à la lecture du paramètre.
        auto tmp_node = std::make_shared<rclcpp::Node>("_kdl_param_reader");
        auto client   = std::make_shared<rclcpp::SyncParametersClient>(
            tmp_node, "robot_state_publisher");

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
        // La cible est exprimée en coordonnées de la base du bras (fr3_link0).
        // Le rail_joint prismatique n'est pas dans cette chaîne → 7 joints mobiles.
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

        ik_vel_    = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
            chain_,
            q_min, q_max,
            *fk_solver_,
            *ik_vel_,
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
        if (!action_client_->wait_for_action_server(std::chrono::seconds(15))) {
            RCLCPP_ERROR(get_logger(),
                "Action server fr3_arm_controller non disponible pour goToReady.");
            publishBusy(false);
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

        // Publier la pose TCP courante dès que KDL est prêt
        if (!kdl_ready_) return;

        int nj = static_cast<int>(chain_.getNrOfJoints());
        KDL::JntArray q(nj);
        for (int i = 0; i < nj; ++i) q(i) = current_q_[i];

        KDL::Frame tcp;
        fk_solver_->JntToCart(q, tcp);

        double qx, qy, qz, qw;
        tcp.M.GetQuaternion(qx, qy, qz, qw);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = this->get_clock()->now();
        ps.header.frame_id = "fr3_link0";
        ps.pose.position.x = tcp.p.x();
        ps.pose.position.y = tcp.p.y();
        ps.pose.position.z = tcp.p.z();
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        pub_tcp_pose_->publish(ps);
    }

    // ====================================================================
    // Callback : réception d'une pose cible
    // ====================================================================

    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!kdl_ready_) {
            RCLCPP_WARN(get_logger(), "Pose reçue mais KDL non initialisé. Ignorer.");
            publishBusy(false);
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Action server fr3_arm_controller non disponible.");
            publishBusy(false);
            return;
        }

        // ── Frame KDL cible ─────────────────────────────────────────────
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;

        // Pose reçue directement en frame fr3_link0.
        // La chaîne KDL part de fr3_link0 → pas de conversion nécessaire.
        KDL::Frame target;
        target.p = KDL::Vector(p.x, p.y, p.z);
        target.M = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);

        RCLCPP_INFO(get_logger(),
            "Pose cible (fr3_link0) : pos=(%.3f, %.3f, %.3f)  quat=(%.3f, %.3f, %.3f, %.3f)",
            p.x, p.y, p.z, q.x, q.y, q.z, q.w);

        // ── IK — multi-seed : minimise le déplacement articulaire ───────
        // On essaie plusieurs seeds et on garde la solution la plus proche
        // de la configuration courante en espace articulaire.
        // Cela évite les sauts de branche (coude haut/bas) quand
        // seule l'orientation change.
        int nj = static_cast<int>(chain_.getNrOfJoints());

        // Seeds à essayer dans l'ordre
        const double* SEEDS[] = { current_q_, Q_HOME, Q_READY };
        const char*   SEED_NAMES[] = { "courante", "home", "ready" };
        constexpr int N_SEEDS = 3;

        KDL::JntArray q_seed(nj), q_out(nj);
        KDL::JntArray best_q(nj);
        double best_dist = std::numeric_limits<double>::max();
        bool   found     = false;

        for (int s = 0; s < N_SEEDS; ++s) {
            for (int i = 0; i < nj; ++i) q_seed(i) = SEEDS[s][i];
            KDL::JntArray q_candidate(nj);
            int ret = ik_solver_->CartToJnt(q_seed, target, q_candidate);
            if (ret < 0) continue;

            // Vérification position (rejeter les solutions divergentes)
            KDL::Frame fk_tmp;
            fk_solver_->JntToCart(q_candidate, fk_tmp);
            double pos_err = (fk_tmp.p - target.p).Norm();
            if (pos_err > 0.003) {          // 3 mm — seuil strict
                RCLCPP_DEBUG(get_logger(),
                    "Seed '%s' : solution rejetée (err pos = %.4f m)",
                    SEED_NAMES[s], pos_err);
                continue;
            }

            // Distance articulaire à la config courante
            double dist = 0.0;
            for (int i = 0; i < nj; ++i) {
                double d = q_candidate(i) - current_q_[i];
                dist += d * d;
            }

            if (!found || dist < best_dist) {
                best_dist = dist;
                best_q    = q_candidate;
                found     = true;
                RCLCPP_DEBUG(get_logger(),
                    "Seed '%s' : IK ok, err=%.4f m, dist_q=%.4f",
                    SEED_NAMES[s], pos_err, std::sqrt(dist));
            }
        }

        if (!found) {
            RCLCPP_ERROR(get_logger(),
                "IK impossible depuis tous les seeds. "
                "Pose hors espace de travail ou orientation singulière.");
            publishBusy(false);
            return;
        }
        q_out = best_q;

        // ── Vérification FK ─────────────────────────────────────────────
        KDL::Frame fk_check;
        fk_solver_->JntToCart(q_out, fk_check);
        double pos_err = (fk_check.p - target.p).Norm();
        RCLCPP_INFO(get_logger(),
            "IK reussie. TCP : (%.3f, %.3f, %.3f)  erreur pos : %.4f m  "
            "dist articulaire : %.4f rad",
            fk_check.p.x(), fk_check.p.y(), fk_check.p.z(),
            pos_err, std::sqrt(best_dist));

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
    std::unique_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_solver_;

    double current_q_[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

    double move_duration_;
    int    ik_max_iter_;
    double ik_tolerance_;
    double rail_position_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    sub_joints_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pub_busy_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pub_tcp_pose_;
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
