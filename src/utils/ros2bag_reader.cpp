#include "utils/ros2bag_reader.hpp"

namespace fs = std::filesystem;

namespace readbag {
    Ros2bagReader::Ros2bagReader() {
    }

    std::vector<std::string> Ros2bagReader::getBagFiles(const std::string& root_folder) {
        /*
            Provided a root_folder, this will check inside all the folders inside the root_folder
            for files of extension ".db3" which is the default ros2 bag extension
        */
        std::vector<std::string> bag_folders;

        for (const auto& entry : fs::directory_iterator(root_folder)) {
            if (fs::is_directory(entry.path())) {
                for (const auto& sub_entry : fs::directory_iterator(entry.path())) {
                    if (sub_entry.path().extension() == ".db3" || sub_entry.path().extension() == ".mcap") {
                        bag_folders.push_back(entry.path().string());
                        break;
                    }
                }
            }
        }

        std::sort(bag_folders.begin(), bag_folders.end());
        return bag_folders;
    }

    std::size_t Ros2bagReader::getMessageCount(const std::string &bag_filepath, const std::string &topic_name, const std::size_t &nth) {
        if (nth <= 0) {
            throw std::invalid_argument("nth must be greater than 0");
        }
        const auto metadata = rosbag2_storage::MetadataIo().read_metadata(bag_filepath);

        for (const auto &topic_info : metadata.topics_with_message_count)
        {
            if (topic_info.topic_metadata.name == topic_name)
            {
                const std::size_t raw_count = static_cast<std::size_t>(topic_info.message_count);
                return (raw_count + nth - 1) / nth;
            }
        }
        return 0;
    }

    std::size_t Ros2bagReader::getTotalMessageCount(const std::string &bag_folder, const std::string &topic_name, const std::size_t &nth) {
        if (nth <= 0) {
            throw std::invalid_argument("nth must be greater than 0");
        }
        const auto bag_files = getBagFiles(bag_folder);

        std::size_t total = 0;

        for (const auto &bag_file : bag_files)
        {
            try
            {
                total += getMessageCount(bag_file, topic_name, nth);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed counting messages in: " << bag_file
                        << " (" << e.what() << ")\n";
            }
        }

        return total; 
    }  
}  // namespace readbag
