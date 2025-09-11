#include "rclcpp/rclcpp.hpp"

// Class Declaration
class MyNode: public rclcpp::Node   //This declares a class named MyNode that inherits from rclcpp::Node
{
    public:
    MyNode(): Node("cpp_test"), counter_(0)     // Is the constructor for the MyNode class
    { 
        // Logs an informational message "Hello Cpp Node" using the ROS 2 logging mechanism
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node"); 

        // Sets up a timer to call timerCallback every second.
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyNode::timerCallback, this));
    }

    private:
        // Increments and logs a counter value in the timerCallback method
        void timerCallback(){
            counter_++;
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
        }

        rclcpp::TimerBase::SharedPtr timer_;    //A shared pointer to the timer object.
        int counter_;        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   //Initialize the node and ROS2 communication
    //Creates a node with shared pointer inside the executable
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node); //Pause, give spin (useful with callbacks)
    rclcpp::shutdown(); //Shutdown the ROS2 communication
    return 0;
}


/* Notes
        class MyNode: public rclcpp::Node 
        MyNode(): Node("cpp_test"), counter_(0)
    It initializes the base class (rclcpp::Node) with the name "cpp_test". 
    This means that the ROS node will be named "cpp_test".
    It also initializes counter_ to 0.
        rclcpp::init(argc, argv); 
    It takes in two arguments: argc, which stands for "argument count" 
    and represents the number of command-line arguments passed into our program, 
    and argv, which stands for "argument vector" 
    and contains an array of strings representing each argument passed in.
        auto node = std::make_shared<rclcpp::Node>("cpp_test"); 
    Next, we create a shared pointer called "node" which points to an instance of rclcpp::Node.
    This creates a new node with the name "cpp_test".
*/