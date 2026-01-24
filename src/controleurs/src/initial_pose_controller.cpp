#include <cmath>
#include <vector>
#include <string>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>

class PoseInitiale : public rclcpp::Node {
public:
    PoseInitiale() : Node("initial_pose_controller") {
        // Paramètres de la pose cible [x, y, z, alpha, beta, gamma]
        target_x_     = this->declare_parameter<double>("target_x", 0.5);
        target_y_     = this->declare_parameter<double>("target_y", 1.0);
        target_z_     = this->declare_parameter<double>("target_z", 1.0);
        target_alpha_ = this->declare_parameter<double>("target_alpha", 0.0);    // roll (rad)
        target_beta_  = this->declare_parameter<double>("target_beta", M_PI);    // pitch (rad)
        target_gamma_ = this->declare_parameter<double>("target_gamma", 0.0);    // yaw (rad)
        
        // Fréquence de publication
        rate_ = this->declare_parameter<double>("rate", 50.0);
        
        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // Noms des joints
        joint_names_ = {"rail_joint", "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
        
        // Initialisation des positions articulaires
        rail_position_ = 0.0;
        arm_joints_.assign(7, 0.0);
        
        // Attendre que robot_state_publisher soit prêt
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // Initialiser la chaîne cinématique KDL
        if (!initKDL()) {
            RCLCPP_ERROR(this->get_logger(), "Échec de l'initialisation KDL!");
            return;
        }
        
        // Résoudre l'IK pour la pose cible
        if (!computeIKForTargetPose()) {
            RCLCPP_ERROR(this->get_logger(), "Impossible d'atteindre la pose cible!");
            return;
        }
        
        // Timer pour publier les joint states
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_),
            std::bind(&PoseInitiale::publishJointStates, this)
        );
        
        RCLCPP_INFO(this->get_logger(), 
            "pose_initiale démarré - TCP cible: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            target_x_, target_y_, target_z_, target_alpha_, target_beta_, target_gamma_);
    }

private:
    
    /**
     * Initialise la chaîne cinématique KDL depuis l'URDF
     * @return true si succès, false sinon
     */
    bool initKDL(){

        // 1. Récupérer le paramètre robot_description depuis robot_state_publisher
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
            this, "robot_state_publisher"
        );
        
        RCLCPP_INFO(this->get_logger(), "Attente de robot_state_publisher...");
        if (!parameters_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), 
                "robot_state_publisher non disponible après 10s");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Récupération de robot_description...");
        auto params = parameters_client->get_parameters({"robot_description"});
        
        if (params.empty() || params[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
            RCLCPP_ERROR(this->get_logger(), 
                "Paramètre robot_description non défini");
            return false;
        }
        
        std::string urdf_xml = params[0].as_string();
        RCLCPP_INFO(this->get_logger(), "URDF récupéré (%zu octets)", urdf_xml.size());
        
        // 2. Parser l'URDF en arbre KDL
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(urdf_xml, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Échec du parsing URDF");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "URDF parsé avec succès");
        
        // 3. Extraire la chaîne cinématique de fr3_link0 à fr3_link8 (TCP)
        if (!tree.getChain("fr3_link0", "fr3_link8", chain_)) {
            RCLCPP_ERROR(this->get_logger(), 
                "Impossible d'extraire la chaîne fr3_link0 → fr3_link8");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Chaîne cinématique extraite: %d joints", chain_.getNrOfJoints());
        
        // 4. Créer le solveur de cinématique directe (FK)
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        
        // 5. Définir les limites articulaires du FR3
        KDL::JntArray q_min(7), q_max(7);
        q_min.data << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        q_max.data << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
        
        // 6. Créer les solveurs de cinématique inverse (IK)
        auto ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
            chain_,
            q_min,              // Limites min
            q_max,              // Limites max
            *fk_solver_,        // Solveur FK
            *ik_vel_solver,     // Solveur IK vitesse
            100,                // Max itérations
            1e-6                // Précision (m)
        );
        
        RCLCPP_INFO(this->get_logger(), "Solveurs KDL initialisés avec succès");
        return true;

    }
    
    /**
     * Calcule la cinématique inverse pour atteindre la pose cible
     * Met à jour rail_position_ et arm_joints_
     * @return true si solution trouvée, false sinon
     */
    bool computeIKForTargetPose(){

      RCLCPP_INFO(this->get_logger(), 
        "Calcul IK pour pose cible: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        target_x_, target_y_, target_z_, target_alpha_, target_beta_, target_gamma_);
    
        // 1. Créer la frame KDL cible (position + orientation)
        KDL::Frame target_frame;
        
        // Position
        target_frame.p = KDL::Vector(target_x_, target_y_, target_z_);
        
        // Orientation (Roll-Pitch-Yaw)
        target_frame.M = KDL::Rotation::RPY(target_alpha_, target_beta_, target_gamma_);
        
        // 2. Configuration initiale pour l'IK (seed)
        //    Utiliser une pose "home" connue du FR3
        KDL::JntArray q_init(7);
        q_init.data << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
        
        // 3. Résoudre l'IK pour le bras (sans le rail)
        KDL::JntArray q_out(7);
        int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_out);
        
        if (ik_result < 0) {
            RCLCPP_ERROR(this->get_logger(), 
                "IK a échoué (code: %d) - pose inatteignable par le bras seul", ik_result);
            
            // Essayer différentes configurations initiales
            RCLCPP_INFO(this->get_logger(), "Tentative avec d'autres seeds...");
            
            // Seed 2: bras étendu
            q_init.data << 0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.785;
            ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_out);
            
            if (ik_result < 0) {
                // Seed 3: configuration repliée
                q_init.data << 0.0, -1.571, 0.0, -2.618, 0.0, 0.785, 0.785;
                ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_out);
            }
            
            if (ik_result < 0) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Toutes les tentatives IK ont échoué - impossible d'atteindre cette pose");
                return false;
            }
        }
        
        // 4. Stocker la solution des joints du bras
        for (int i = 0; i < 7; ++i) {
            arm_joints_[i] = q_out(i);
        }
        
        // 5. Position du rail
        //    Pour l'instant: rail fixe à 0.0
        //    TODO: optimiser la position du rail pour maximiser la manipulabilité
        rail_position_ = 0.0;
        
        // 6. Vérifier la solution avec FK
        KDL::Frame fk_result;
        fk_solver_->JntToCart(q_out, fk_result);
        
        // Calculer l'erreur de position
        double error_x = fk_result.p.x() - target_frame.p.x();
        double error_y = fk_result.p.y() - target_frame.p.y();
        double error_z = fk_result.p.z() - target_frame.p.z();
        double position_error = std::sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
        
        // Calculer l'erreur d'orientation (angle entre les rotations)
        KDL::Rotation error_rot = target_frame.M * fk_result.M.Inverse();

        KDL::Vector rotation_axis;
        double orientation_error = error_rot.GetRotAngle(rotation_axis);
        
        RCLCPP_INFO(this->get_logger(), 
            "✓ IK réussie! Erreur position: %.6f m, orientation: %.6f rad", 
            position_error, orientation_error);
        
        // Afficher la configuration trouvée
        RCLCPP_INFO(this->get_logger(), 
            "Configuration: rail=%.3f, q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            rail_position_, 
            arm_joints_[0], arm_joints_[1], arm_joints_[2], arm_joints_[3],
            arm_joints_[4], arm_joints_[5], arm_joints_[6]);
        
        // 7. Vérifier que l'erreur est acceptable
        if (position_error > 0.001 || orientation_error > 0.01) {
            RCLCPP_WARN(this->get_logger(), 
                "⚠ Erreur IK importante! Position: %.6f m, Orientation: %.6f rad", 
                position_error, orientation_error);
        }
        
        return true;

    }
    
    /**
     * Publie les états articulaires (rail + bras) sur /joint_states
     */
    void publishJointStates(){
        sensor_msgs::msg::JointState msg;
    
        // Timestamp
        msg.header.stamp = this->now();
        
        // Noms des joints
        msg.name = joint_names_;
        
        // Positions: rail + 7 joints du bras
        msg.position.reserve(8);
        msg.position.push_back(rail_position_);  // rail_joint
        msg.position.insert(msg.position.end(), arm_joints_.begin(), arm_joints_.end());  // joint1-7
        
        // Publier
        pub_->publish(msg);
    }
    
    /**
     * (Future) Lit la pose cible depuis un fichier
     * @param filename chemin du fichier
     * @return true si lecture réussie
     */
    //bool loadTargetPoseFromFile(const std::string& filename);
    
    // === Membres privés ===
    
    // Pose cible du TCP
    double target_x_, target_y_, target_z_;
    double target_alpha_, target_beta_, target_gamma_;
    
    // Configuration articulaire solution
    double rail_position_;
    std::vector<double> arm_joints_;  // 7 joints du bras
    
    // Paramètres
    double rate_;
    std::vector<std::string> joint_names_;
    
    // KDL
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
    
    // ROS 2
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseInitiale>());
    rclcpp::shutdown();
    return 0;
}