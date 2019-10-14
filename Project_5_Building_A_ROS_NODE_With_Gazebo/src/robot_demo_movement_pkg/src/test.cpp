int main( int argc, char **argv )
{
    // Initialize the ROS system
    ros::init( argc, argv, "elikos_aiAgent" );
 
    // Establish this program as a ROS node
    ros::NodeHandle nh;
    ros::Rate r(30);    //10 hz
 
    // Create the quad AI Agent
    elikos_ai::Agent agent(&nh);
    agent.init();
 
    while (ros::ok())
    {
         
        // stuff
 
        ros::spinOnce(); // the missing call
        r.sleep();
    }
 
    agent.destroy();
}
 
class Agent
{
 
public:
 
    Agent( ros::NodeHandle* nh );
    ~Agent();
    void init();
    void destroy();
    void run();
 
    void receiveRobotsPosCallback( const elikos_ros::RobotsPos& msg );
 
private:
    ros::NodeHandle& nh_;
 
    void setSubscribers();
 
    std::map<std::string, ros::Publisher> rosPublishers_;
     std::map<std::string, ros::Subscriber> rosSubscribers_;
 
    // I've tested with this version too (the subscriber not being in a map)
    // ros::Subscriber sub_;
};
 
Agent::Agent( ros::NodeHandle* nh ) : nh_(*nh)
{}
 
void Agent::init()
{
    setSubscribers();
}
 
void Agent::setSubscribers()
{
    ROS_INFO_STREAM( "Agent::setSubscribers" );
    // Subscribe to all robots' positions' topics
    std::string robotsPosTopic = TOPICS_NAMES[robotsPos];
     ros::Subscriber sub =  nh_.subscribe(robotsPosTopic, 1000,  &Agent::receiveRobotsPosCallback, this );
    rosSubscribers_.insert( std::pair<std::string,ros::Subscriber>(robotsPosTopic, sub) );
    ROS_INFO_STREAM( "Agent::setSubscribers, end" );
 
    // I've tested with this version too (the subscriber not being in a map)
    // sub_ = nh_.subscribe(robotsPosTopic, 1000, &Agent::receiveRobotsPosCallback, this );
}
 
void Agent::receiveRobotsPosCallback( const elikos_ros::RobotsPos& msg )
{
    std::cout << "AGENT CALLBACK" << std::endl;
    ROS_INFO_STREAM( "Agent::callback -- Push RobotsPos message" );
    queueRobotsPos_.push( msg );
}