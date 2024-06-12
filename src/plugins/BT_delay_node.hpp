#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <thread>

class DelayNodeBT : public BT::SyncActionNode
{
public:
    DelayNodeBT(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {

    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("duration") };
    }

    BT::NodeStatus tick() override
    {
        double duration;
        if (!getInput<double>("duration", duration))
        {
            duration = delay_duration_; // Use the parameter if the input port is not set
        }
        std::cout<<"Delay for" << duration;
        // Sleep for the specified duration
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        return BT::NodeStatus::SUCCESS;
    }

private:
    double delay_duration_;
};
