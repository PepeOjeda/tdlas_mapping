#include <tdlas_mapping/reactive_master.h>
#include <tdlas_mapping/reactive_follower.h>
#include <ament_imgui/ament_imgui.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <PoseJSON.hpp>

using KeyValue = diagnostic_msgs::msg::KeyValue;

class Reactive : public rclcpp::Node
{
    public:
    std::shared_ptr<ReactiveMaster> master;
    rclcpp::Subscription<KeyValue>::SharedPtr mqttSub;


    Reactive() : Node("Reactive")
    {
        using namespace std::placeholders;
        mqttSub = create_subscription<KeyValue>("/mqtt2ros", 1, std::bind(&Reactive::mqttCallback, this, std::placeholders::_1));
        
    }

    bool shouldRun = false;
    geometry_msgs::msg::PoseStamped goal_pose;
    void mqttCallback(KeyValue::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "%s", msg->value.c_str());
    
        if(msg->key == "/reactive")
        {
            RCLCPP_INFO(get_logger(), "Reactive goal received, starting");
            auto json = nlohmann::json::parse(msg->value);
            goal_pose = nav2MQTT::from_json(json);
            shouldRun = true;
        }
    }


    void execute()
    {
        master->run(goal_pose);    
        shouldRun = false;    
    }

    void render()
    {
        // may throw ament_index_cpp::PackageNotFoundError exception
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("tdlas_mapping");
        AMENT_IMGUI::setup( (package_share_directory+"/imgui.ini").c_str() );
        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            AMENT_IMGUI::StartFrame();
            master->render();
            rate.sleep();
            AMENT_IMGUI::Render();
        }
        AMENT_IMGUI::close();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Reactive> node = std::make_shared<Reactive>();
    node->master = std::make_shared<ReactiveMaster>(node);
    
    std::thread renderThread(&Reactive::render, node.get());

    rclcpp::Rate rate(20);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);   
        if(node->shouldRun)
            node->execute();    
        rate.sleep();
    }
    renderThread.join();
}