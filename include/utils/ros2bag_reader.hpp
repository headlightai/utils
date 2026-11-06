#pragma once

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace readbag {
template <typename T>
struct StampedMsg {
    T msg;
    int64_t stamp_ns;  // bag send timestamp (nanoseconds); equals the receive timestamp if unavailable
};

class Ros2bagReader {
   public:
    typedef std::shared_ptr<Ros2bagReader> Ptr;

    Ros2bagReader();

    ~Ros2bagReader() = default;

    template <typename T>
    std::vector<T> readMsg(const std::string& bag_folder, const std::string& topic_name, const int& nth = 1);

    template <typename T>
    std::optional<StampedMsg<T>> readFirstMsg(const std::string& bag_folder, const std::string& topic_name);

    std::vector<std::string> getBagFiles(const std::string& root_folder);

    std::vector<std::string> getBagFileNamesFromMetadata(const std::string& root_folder);

    rosbag2_storage::BagMetadata getBagMetadata(const std::string& bag_folder);

    std::size_t getMessageCount(const std::string& bag_filepath, const std::string& topic_name,
                                const std::size_t& nth = 1);

    std::size_t getTotalMessageCount(const std::string& bag_folder, const std::string& topic_name,
                                     const std::size_t& nth = 1);

    int64_t getBagDuration(const std::string& bag_file);

   private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

    rosbag2_storage::StorageOptions storage_options_;
};

template <typename T>
std::vector<T> Ros2bagReader::readMsg(const std::string& bag_folder, const std::string& topic_name, const int& nth) {
    /*
        bag_file: path to the ros2 bag folder
        topic_name: the specific topic name that is to be returned as a ros2 msg
        For example, this will read every nth msg under the topic_name in a ros2bag, stores in an
       array and returns it.
    */
    storage_options_.uri = bag_folder;
    storage_options_.storage_id = "";
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options_);
    reader_->open(storage_options_);

    std::vector<T> msg_array;
    int message_count = 0;
    std::shared_ptr<T> ros_msg = std::make_shared<T>();
    rclcpp::Serialization<T> serialization;

    while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        if (msg->topic_name == topic_name) {
            if (message_count % nth == 0) {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                serialization.deserialize_message(&serialized_msg, ros_msg.get());

                msg_array.emplace_back(*ros_msg);
            }
            message_count++;
        }
    }
    return msg_array;
}

template <typename T>
std::optional<StampedMsg<T>> Ros2bagReader::readFirstMsg(const std::string& bag_folder, const std::string& topic_name) {
    /*
        Returns the first msg on topic_name in the bag along with its send timestamp, or std::nullopt
        if the topic has no msgs. Stops reading as soon as the msg is found.
    */
    storage_options_.uri = bag_folder;
    storage_options_.storage_id = "";
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options_);
    reader_->open(storage_options_);

    rclcpp::Serialization<T> serialization;
    while (reader_->has_next()) {
        const auto msg = reader_->read_next();
        if (msg->topic_name == topic_name) {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            StampedMsg<T> stamped{T{}, msg->send_timestamp};
            serialization.deserialize_message(&serialized_msg, &stamped.msg);
            return stamped;
        }
    }
    return std::nullopt;
}
}  // namespace readbag