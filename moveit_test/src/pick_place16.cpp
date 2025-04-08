#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <franka_gripper/GraspActionGoal.h>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>

struct DetectedObject {
    geometry_msgs::PoseStamped pose;
    std::string object_class;

     // Define equality operator for DetectedObject
     //bool operator==(const DetectedObject& other) const {
     //   return pose == other.pose && object_class == other.object_class;
    //}
};

std::vector<DetectedObject> detected_objects;
int total_detected_objects = 0;
bool new_object_received = false;
bool stop_receiving = false; // Flag to stop receiving objects
bool ready_for_next_object = true; // Flag to indicate readiness for the next object

ros::Subscriber sub; // Declare subscriber globally to unsubscribe later

void detectedObjectsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (stop_receiving) return; // Stop processing if we have enough objects

    DetectedObject new_object;
    new_object.pose = *msg;
    new_object.object_class = msg->header.frame_id;

    detected_objects.push_back(new_object);
    new_object_received = true;

    // Check if we have received enough objects
    if ((detected_objects.size() >= total_detected_objects)&&(total_detected_objects != 0)) {
        stop_receiving = true;
        sub.shutdown(); // Unsubscribe from the topic
        ROS_INFO("Received %d objects. Stopping subscription to /object_poses.", total_detected_objects);
    }
}

void objectsNumberCallback(const std_msgs::String::ConstPtr& msg) {
    std::stringstream ss(msg->data);
    ss >> total_detected_objects;
    //ROS_INFO("Total objects to detect: %d", total_detected_objects);
}

bool transformPose(const tf2_ros::Buffer& tf_buffer, 
                   const geometry_msgs::PoseStamped& pose_in_camera, 
                   geometry_msgs::PoseStamped& pose_in_panda) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("panda_link0", "camera_link", ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(pose_in_camera, pose_in_panda, transform);
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("TF transformation failed: %s", ex.what());
        return false;
    }
}

void publishGraspGoal(ros::Publisher &publisher, double width) {
    franka_gripper::GraspActionGoal goal_msg;
    goal_msg.goal.width = width;
    goal_msg.goal.epsilon.inner = 0.005;
    goal_msg.goal.epsilon.outer = 0.005;
    goal_msg.goal.speed = 0.1;
    goal_msg.goal.force = 5.0;

    publisher.publish(goal_msg);
    ROS_INFO("Published grasp goal with width: %.3f, speed: %.2f, force: %.2f", 
             goal_msg.goal.width, goal_msg.goal.speed, goal_msg.goal.force);
}

void moveToNewPosition(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped& pose_in_panda) {
    geometry_msgs::Pose target_pose = pose_in_panda.pose;
    target_pose.position.z += 0.1;

    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(20.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = move_group.plan(plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
    } else {
        ROS_ERROR("Planning failed.");
    }
}

void goHome(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::vector<double> home_joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    move_group.setJointValueTarget(home_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = move_group.plan(plan);

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        ROS_INFO("Robot returned to home position.");
    } else {
        ROS_ERROR("Failed to plan to home position.");
    }
}

void setGripperForce(moveit::planning_interface::MoveGroupInterface& gripper_group, double force) {
    std::vector<double> joint_values = gripper_group.getCurrentJointValues();
    std::map<std::string, double> joint_efforts;
    joint_efforts["panda_finger_joint_l"] = force;
    joint_efforts["panda_finger_joint_r"] = force;

    gripper_group.setJointValueTarget(joint_values);
    gripper_group.setMaxVelocityScalingFactor(0.05);
    gripper_group.setMaxAccelerationScalingFactor(0.1);

    gripper_group.move();
    ROS_INFO("Gripper force set to: %f", force);
}

void openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group) {
    gripper_group.setNamedTarget("open");
    gripper_group.move();
    ROS_INFO("Gripper opened.");
}

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double force = 50.0) {
    std::vector<double> joint_values = {0.03, 0.03};
    gripper_group.setJointValueTarget(joint_values);

    std::map<std::string, double> joint_efforts;
    joint_efforts["panda_finger_joint_l"] = force;
    joint_efforts["panda_finger_joint_r"] = force;

    gripper_group.setMaxVelocityScalingFactor(0.05);
    gripper_group.setMaxAccelerationScalingFactor(0.1);

    gripper_group.move();
    ROS_INFO("Gripper closed with increased force.");

    std::vector<double> current_joint_values = gripper_group.getCurrentJointValues();
    ROS_INFO("Current gripper joint values: %f, %f", current_joint_values[0], current_joint_values[1]);
}

void pickObject(moveit::planning_interface::MoveGroupInterface& move_group, 
    const geometry_msgs::PoseStamped& pose_in_panda,
    moveit::planning_interface::MoveGroupInterface& gripper_group, double grasp_force,
    ros::Publisher& publisher, const std::string& object_class) {
geometry_msgs::PoseStamped pre_grasp_pose = pose_in_panda;
pre_grasp_pose.pose.position.z += 0.1;
moveToNewPosition(move_group, pre_grasp_pose);

openGripper(gripper_group);

moveToNewPosition(move_group, pose_in_panda);

double width;
if (object_class == "green" || object_class == "blue") {
width = 0.045;
} else if (object_class == "red") {
width = 0.065;
} else {
width = 0.045; // Default width
}

publishGraspGoal(publisher, width);

ros::Duration(2).sleep();

pre_grasp_pose.pose.position.z += 0.1;
moveToNewPosition(move_group, pre_grasp_pose);
}

void placeObject(moveit::planning_interface::MoveGroupInterface& move_group,
                 const geometry_msgs::PoseStamped& pose_in_panda,
                 const DetectedObject& object,
                 moveit::planning_interface::MoveGroupInterface& gripper_group,
                ros::NodeHandle& nh) {
    geometry_msgs::PoseStamped place_pose;

    ROS_INFO("place object class: %s", object.object_class.c_str());

    if (object.object_class.c_str()[0] == 'r') {
        place_pose.pose.position.x = 0.0;
        place_pose.pose.position.y = 0.5;
        place_pose.pose.position.z = 0.3;
        place_pose.pose.orientation.x = 1;
        place_pose.pose.orientation.y = 0;
        place_pose.pose.orientation.z = 0;
        place_pose.pose.orientation.w = 0;
    } else if (object.object_class.c_str()[0] == 'b') {
        place_pose.pose.position.x = 0.2;
        place_pose.pose.position.y = 0.5;
        place_pose.pose.position.z = 0.3;
        place_pose.pose.orientation.x = 1;
        place_pose.pose.orientation.y = 0;
        place_pose.pose.orientation.z = 0;
        place_pose.pose.orientation.w = 0;
    } else if (object.object_class.c_str()[0] == 'g') {
        place_pose.pose.position.x = 0.4;
        place_pose.pose.position.y = 0.5;
        place_pose.pose.position.z = 0.3;
        place_pose.pose.orientation.x = 1;
        place_pose.pose.orientation.y = 0;
        place_pose.pose.orientation.z = 0;
        place_pose.pose.orientation.w = 0;
    } else {
        place_pose.pose.position.x = 0.5;
        place_pose.pose.position.y = 0.5;
        place_pose.pose.position.z = 0.3;
        place_pose.pose.orientation.x = 1;
        place_pose.pose.orientation.y = 0;
        place_pose.pose.orientation.z = 0;
        place_pose.pose.orientation.w = 0;
    }

    geometry_msgs::PoseStamped pre_place_pose = place_pose;
    pre_place_pose.pose.position.z += 0.1;
    moveToNewPosition(move_group, pre_place_pose);

    moveToNewPosition(move_group, place_pose);

    openGripper(gripper_group);

    pre_place_pose.pose.position.z += 0.1;
    moveToNewPosition(move_group, pre_place_pose);
    goHome(move_group);

    // Resubscribe to /object_poses after placing the object
    sub = nh.subscribe<geometry_msgs::PoseStamped>("/object_poses", 1, detectedObjectsCallback);
    ready_for_next_object = true; // Set the flag to indicate readiness for the next object

}

bool compareObjects(const DetectedObject& a, const DetectedObject& b) {
    std::vector<std::string> order = {"green", "blue", "red"};
    auto a_it = std::find(order.begin(), order.end(), a.object_class);
    auto b_it = std::find(order.begin(), order.end(), b.object_class);
    return a_it < b_it;
}

/*bool IsDetectedObjectEmpty(const DetectedObject& emp){
    DetectedObject default_struct = {};
    return emp == default_struct;
}*/

class CollisionObjectAdder {
    public:
        CollisionObjectAdder() : scene_interface_() {}
    
        void addCollisionObjects() {
            // Add workbench
            geometry_msgs::PoseStamped workbench_pose;
            workbench_pose.header.frame_id = "world";
            workbench_pose.pose.position.x = 0.70;
            workbench_pose.pose.position.y = 0.60;
            workbench_pose.pose.position.z = -0.02;
            std::vector<double> workbench_size = {1.8, 1.40, 0.001};  // Box size
            addBox("workbench", workbench_pose, workbench_size);
    
            /*// Add bin bench
            geometry_msgs::PoseStamped bin_bench_pose;
            bin_bench_pose.header.frame_id = "world";
            bin_bench_pose.pose.position.x = -0.55;
            bin_bench_pose.pose.position.y = 0.0;
            bin_bench_pose.pose.position.z = 0.1;
            std::vector<double> binbench_size = {0.4, 1.5, 0.2};  // Box size
            addBox("binbench", bin_bench_pose, binbench_size);*/
    
            waitForObjects();
        }
    
    private:
        void addBox(const std::string& name, const geometry_msgs::PoseStamped& pose, const std::vector<double>& size) {
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = pose.header.frame_id;
            collision_object.id = name;
    
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = size[0];
            primitive.dimensions[1] = size[1];
            primitive.dimensions[2] = size[2];
    
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose.pose);
            collision_object.operation = collision_object.ADD;
    
            scene_interface_.applyCollisionObject(collision_object);
        }
    
        void waitForObjects(int timeout = 5) {
            double start = ros::Time::now().toSec();
            double elapsed_time = 0;
            bool are_objects_known = false;
    
            while (elapsed_time < timeout && ros::ok()) {
                std::vector<std::string> known_objects = scene_interface_.getKnownObjectNames();
                are_objects_known = (std::find(known_objects.begin(), known_objects.end(), "workbench") != known_objects.end());
    
                if (are_objects_known) {
                    return;
                }
    
                ros::Duration(0.1).sleep();
                elapsed_time = ros::Time::now().toSec() - start;
            }
    
            ROS_WARN("Warning! Collision objects not yet visible in the planning scene.");
        }
    
        moveit::planning_interface::PlanningSceneInterface scene_interface_;
    };

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_to_panda_transform");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Subscriber objects_number_sub = nh.subscribe<std_msgs::String>("/objects_number", 1, objectsNumberCallback);    
    ros::Publisher publisher = nh.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", 10);

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    CollisionObjectAdder adder;
    adder.addCollisionObjects();

    DetectedObject obj;

    ros::Rate rate(10);
    while (ros::ok()) {
        if(ready_for_next_object){
            ready_for_next_object = false;
            sub = nh.subscribe<geometry_msgs::PoseStamped>("/object_poses", 1, detectedObjectsCallback);
            
        }
        if (new_object_received && !detected_objects.empty()) {
            new_object_received = false;

            std::sort(detected_objects.begin(), detected_objects.end(), compareObjects);

            obj = detected_objects[0];


            /*for (const auto& obj : detected_objects) {
                ROS_INFO("Detected object class: %s", obj.object_class.c_str());

                geometry_msgs::PoseStamped pose_in_panda;
                if (transformPose(tf_buffer, obj.pose, pose_in_panda)) {
                    ROS_INFO("Pose in panda_link0 frame: x=%f, y=%f, z=%f",
                             pose_in_panda.pose.position.x,
                             pose_in_panda.pose.position.y,
                             pose_in_panda.pose.position.z);

                    pickObject(move_group, pose_in_panda, gripper_group, 5.0, publisher);
                    placeObject(move_group, pose_in_panda, obj, gripper_group);
                } else {
                    ROS_ERROR("Failed to transform pose.");
                }
            }*/
            ROS_INFO("Detected object class: %s", obj.object_class.c_str());

                geometry_msgs::PoseStamped pose_in_panda;
                if (transformPose(tf_buffer, obj.pose, pose_in_panda)) {
                    ROS_INFO("Pose in panda_link0 frame: x=%f, y=%f, z=%f",
                             pose_in_panda.pose.position.x,
                             pose_in_panda.pose.position.y,
                             pose_in_panda.pose.position.z);

                    pickObject(move_group, pose_in_panda, gripper_group, 5.0, publisher,obj.object_class);
                    detected_objects.erase(detected_objects.begin());
                    detected_objects.clear();
                    stop_receiving = false;
                    placeObject(move_group, pose_in_panda, obj, gripper_group, nh);
                    //obj = detected_objects[0];
                } else {
                    ROS_ERROR("Failed to transform pose.");
                }
            //detected_objects.clear();
        }

        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}