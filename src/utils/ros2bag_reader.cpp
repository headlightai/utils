#include "utils/ros2bag_reader.hpp"

namespace fs = std::filesystem;

namespace readbag {
Ros2bagReader::Ros2bagReader() {
}

std::vector<std::string> Ros2bagReader::getBagFiles(const std::string& root_folder) {
    /*
      Supports two layouts:
      1) root_folder is itself a rosbag2 directory (contains metadata.yaml)
      2) root_folder contains multiple bag directories as immediate children
    */
    std::vector<std::string> bag_folders;
    const fs::path root_path(root_folder);

    std::error_code ec;
    if (!fs::exists(root_path, ec) || !fs::is_directory(root_path, ec)) {
        return bag_folders;
    }

    auto has_metadata = [](const fs::path& dir) -> bool {
        std::error_code local_ec;
        const fs::path metadata_path = dir / "metadata.yaml";
        return fs::exists(metadata_path, local_ec) && fs::is_regular_file(metadata_path, local_ec);
    };

    // Case 1: root itself is a bag directory
    if (has_metadata(root_path)) {
        bag_folders.push_back(root_path.string());
        return bag_folders;
    }

    // Case 2: immediate child directories are bag directories
    for (const auto& entry : fs::directory_iterator(root_path, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_directory(ec)) {
            continue;
        }
        if (has_metadata(entry.path())) {
            bag_folders.push_back(entry.path().string());
        }
    }

    std::sort(bag_folders.begin(), bag_folders.end());
    return bag_folders;
}

std::vector<std::string> Ros2bagReader::getBagFileNamesFromMetadata(const std::string& root_folder) {
    const auto metadata = rosbag2_storage::MetadataIo().read_metadata(root_folder);
    std::vector<std::string> bag_file_names;
    for (const auto& relative_path : metadata.relative_file_paths) {
        bag_file_names.push_back(fs::path(root_folder) / relative_path);
    }
    return bag_file_names;
}

std::size_t Ros2bagReader::getMessageCount(const std::string& bag_filepath, const std::string& topic_name,
                                           const std::size_t& nth) {
    if (nth <= 0) {
        throw std::invalid_argument("nth must be greater than 0");
    }
    const auto metadata = rosbag2_storage::MetadataIo().read_metadata(bag_filepath);

    for (const auto& topic_info : metadata.topics_with_message_count) {
        if (topic_info.topic_metadata.name == topic_name) {
            const std::size_t raw_count = static_cast<std::size_t>(topic_info.message_count);
            return (raw_count + nth - 1) / nth;
        }
    }
    return 0;
}

std::size_t Ros2bagReader::getTotalMessageCount(const std::string& bag_folder, const std::string& topic_name,
                                                const std::size_t& nth) {
    if (nth <= 0) {
        throw std::invalid_argument("nth must be greater than 0");
    }
    const auto bag_files = getBagFiles(bag_folder);

    std::size_t total = 0;

    for (const auto& bag_file : bag_files) {
        try {
            total += getMessageCount(bag_file, topic_name, nth);
        } catch (const std::exception& e) {
            std::cerr << "Failed counting messages in: " << bag_file << " (" << e.what() << ")\n";
        }
    }

    return total;
}
}  // namespace readbag
