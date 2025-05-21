#include <filesystem>
#include <map>
#include <regex>
#include <set>
#include <vector>

#include <CLI/CLI.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

struct StringTransform {
  enum class Type { Substitute, Filter, Prefix };

  StringTransform(const std::string& expr, const std::string& sub)
      : expr(std::regex(expr)), sub(sub) {}

  static StringTransform from_arg(Type type,
                                  const std::string& arg,
                                  const rclcpp::Logger& logger) {
    switch (type) {
      case Type::Substitute: {
        const auto pos = arg.find(":");
        if (pos == std::string::npos) {
          RCLCPP_WARN_STREAM(logger, "Invalid substitution: '" << arg << "'");
          return StringTransform("", "");
        }

        const auto expr = arg.substr(0, pos);
        const auto sub = arg.substr(pos + 1);
        return StringTransform(expr, sub);
      }
      case Type::Filter:
        return StringTransform(arg, "");
      case Type::Prefix:
        return StringTransform("^.+", arg + "$&");
      default:
        return StringTransform("", "");
    }
  }

  std::string apply(const std::string& frame_id) const {
    std::regex re(expr);
    return std::regex_replace(frame_id, re, sub);
  }

  const std::regex expr;
  const std::string sub;
};

using rosbag2_transport::ReaderWriterFactory;

struct MessageInfo {
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
  const rosbag2_storage::TopicMetadata* metadata = nullptr;

  operator bool() const { return msg != nullptr && metadata != nullptr; }
};

struct ReaderInfo {
  ReaderInfo(const std::filesystem::path& bagpath) {
    rosbag2_storage::StorageOptions opts;
    opts.uri = bagpath;
    reader = ReaderWriterFactory::make_reader(opts);
    if (!reader) {
      return;
    }

    reader->open(opts);

    const auto metadata = reader->get_all_topics_and_types();
    for (const auto& data : metadata) {
      lookup[data.name] = data;
    }
  }

  operator bool() const { return reader != nullptr; }

  MessageInfo next() const {
    while (reader->has_next()) {
      auto msg = reader->read_next();
      if (!msg) {
        //std::cout << "Reader has invalid message!" << std::endl;
        continue;
      }

      auto iter = lookup.find(msg->topic_name);
      if (iter == lookup.end()) {
        std::cout << "Error: could not find metadata for topic '" << msg->topic_name
                  << "'" << std::endl;
        continue;
      }

      //std::cout << "Got valid message!" << std::endl;
      return {msg, &iter->second};
    }

    //std::cout << "Reader has no messages!" << std::endl;
    return {};
  }

  std::unique_ptr<rosbag2_cpp::Reader> reader;
  std::map<std::string, rosbag2_storage::TopicMetadata> lookup;
};

using ReaderVec = std::vector<ReaderInfo>;
using MsgVec = std::vector<MessageInfo>;

void fill_messages(const ReaderVec& readers, MsgVec& msgs) {
  for (size_t i = 0; i < readers.size(); ++i) {
    if (!readers[i]) {
      //std::cout << "Invalid reader in list: " << i << "!" << std::endl;
      continue;
    }

    if (msgs[i]) {
      //std::cout << "Previous message present: " << i << "!" << std::endl;
      continue;
    }

    //std::cout << "Updated message: " << i << "!" << std::endl;
    msgs[i] = readers[i].next();
  }
}

MessageInfo get_next_message(MsgVec& msgs) {
  size_t best_idx = 0;
  MessageInfo best_info;
  for (size_t i = 0; i < msgs.size(); ++i) {
    const auto& info = msgs[i];
    if (!info) {
      continue;
    }

    if (!best_info || info.msg->recv_timestamp < best_info.msg->recv_timestamp) {
      best_info = info;
      best_idx = i;
    }
  }

  if (best_info) {
    msgs[best_idx] = {};
  }

  return best_info;
}

void merge_bags(const std::vector<std::filesystem::path>& inputs,
                const std::filesystem::path& output_path) {
  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);

  rosbag2_storage::StorageOptions output_options;
  output_options.uri = output_path;
  rosbag2_cpp::writers::SequentialWriter writer;
  writer.open(output_options, rosbag2_cpp::ConverterOptions{});

  ReaderVec readers;
  for (const auto& input : inputs) {
    if (!std::filesystem::exists(input)) {
      std::cout << "Error: invalid bag '" << input << "'" << std::endl;
      continue;
    }

    readers.emplace_back(input);
  }

  std::set<std::string> seen;
  MsgVec msgs(readers.size());
  fill_messages(readers, msgs);
  MessageInfo to_write = get_next_message(msgs);
  while (to_write) {
    if (!seen.count(to_write.msg->topic_name)) {
      writer.create_topic(*to_write.metadata);
      seen.insert(to_write.msg->topic_name);
    }

    writer.write(to_write.msg);
    fill_messages(readers, msgs);
    to_write = get_next_message(msgs);
  }
}

int main(int argc, char** argv) {
  CLI::App app("Utility to merge multiple rosbags");
  argv = app.ensure_utf8(argv);
  app.allow_extras();

  std::filesystem::path from_bag;
  std::filesystem::path to_bag;
  app.add_option("from_bag", from_bag)
      ->check(CLI::ExistingPath)
      ->description("bag to take topics from");
  app.add_option("to_bag", to_bag)
      ->check(CLI::ExistingPath)
      ->description("bag to write topics to");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  std::filesystem::path output;
  if (std::filesystem::is_directory(to_bag)) {
    output = to_bag.parent_path();
  } else {
    output = to_bag.parent_path().parent_path();
  }

  const std::string output_name = "." + to_bag.stem().string();
  output /= output_name;
  std::cout << "Merging " << from_bag << " -> " << to_bag << " @ " << output
            << std::endl;
  merge_bags({from_bag, to_bag}, output);
  return 0;
}
