#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcMergeOffline{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
        ros::Publisher debug_pub_;
        /*buffer*/
        struct PcTopic{
            std::string topic_name;
            sensor_msgs::PointCloud2ConstPtr pc_ptr;
            bool is_buffered = false;
        };
        std::vector<PcTopic> pc_topic_list_;
        rosbag::Bag save_bag_;
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string save_topic_name_;
		std::string debug_frame_;
        float debug_hz_;
        /*function*/
        void openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode);
        bool isBuffered();
        void mergePc(sensor_msgs::PointCloud2& ros_merged_pc);

	public:
		PcMergeOffline();
        void execute();
};

PcMergeOffline::PcMergeOffline()
	: nh_private_("~")
{
	std::cout << "----- pc_projection_to_image_offline -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_merged.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;
    nh_private_.param("save_topic_name", save_topic_name_, std::string("/merged_pc"));
	std::cout << "save_topic_name_ = " << save_topic_name_ << std::endl;
    nh_private_.param("debug_frame", debug_frame_, std::string("debug"));
	std::cout << "debug_frame_ = " << debug_frame_ << std::endl;
    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;
    for(size_t i = 0; ; i++){
        PcTopic tmp_pc_topic;
        if(!nh_private_.getParam("pc_" + std::to_string(i), tmp_pc_topic.topic_name))  break;
        pc_topic_list_.push_back(tmp_pc_topic);
        std::cout << "pc_topic_list_[" << i << "].topic_name = " << pc_topic_list_[i].topic_name << std::endl;
    }

    /*publisher*/
    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(save_topic_name_, 1);

    /*initialize*/
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
    openRosBag(save_bag_, save_rosbag_path_, rosbag::bagmode::Append);
}

void PcMergeOffline::openRosBag(rosbag::Bag& bag, const std::string& rosbag_path, int mode)
{
    try{
        bag.open(rosbag_path, mode);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path << std::endl;
        exit(true);
    }
}

void PcMergeOffline::execute()
{
    rosbag::Bag load_bag;
    openRosBag(load_bag, load_rosbag_path_, rosbag::bagmode::Read);

    rosbag::View view;
    view.addQuery(load_bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        for(PcTopic& pc_topic : pc_topic_list_){
            if(view_itr->getTopic() == pc_topic.topic_name){
                pc_topic.pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                pc_topic.is_buffered = true;
                break;
            }
        }
        if(isBuffered()){
            sensor_msgs::PointCloud2 ros_merged_pc;
            mergePc(ros_merged_pc);
            save_bag_.write(save_topic_name_, view_itr->getTime(), ros_merged_pc);
            for(PcTopic& pc_topic : pc_topic_list_)    pc_topic.is_buffered = false;
            ros_merged_pc.header.frame_id = debug_frame_;
            debug_pub_.publish(ros_merged_pc);
            if(debug_hz_ > 0)    loop_rate.sleep();
        }
        view_itr++;
    }
    load_bag.close();
    save_bag_.close();
}

bool PcMergeOffline::isBuffered()
{
    for(PcTopic& pc_topic : pc_topic_list_){
        if(!pc_topic.is_buffered)  return false;
    }
    return true;
}

void PcMergeOffline::mergePc(sensor_msgs::PointCloud2& ros_merged_pc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_merged_pc (new pcl::PointCloud<pcl::PointXYZ>);
    for(PcTopic& pc_topic : pc_topic_list_){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_topic.pc_ptr, *pcl_pc);
        if(pcl_merged_pc->points.empty())   *pcl_merged_pc = *pcl_pc;
        else    *pcl_merged_pc += *pcl_pc;
    }
    pcl::toROSMsg(*pcl_merged_pc, ros_merged_pc);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_merge_offline");
	
	PcMergeOffline pc_merge_offline;
    pc_merge_offline.execute();
}