#include "ianvs/bag_reader.h"

namespace ianvs {

BagReader::BagReader(const std::filesystem::path& bagpath) {
  rosbag2_storage::StorageOptions opts;
  opts.uri = bagpath;
  reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
  if (!reader) {
    return;
  }

  reader->open(opts);

  const auto metadata = reader->get_all_topics_and_types();
  for (const auto& data : metadata) {
    lookup[data.name] = std::make_shared<rosbag2_storage::TopicMetadata>(data);
  }
}

MessageInfo BagReader::next() const {
  while (reader->has_next()) {
    auto msg = reader->read_next();
    if (!msg) {
      continue;
    }

    auto iter = lookup.find(msg->topic_name);
    if (iter == lookup.end()) {
      continue;
    }

    return {msg, iter->second};
  }

  return {};
}

}  // namespace ianvs
