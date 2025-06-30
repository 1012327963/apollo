/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <csignal>
#include <iostream>

#include "cyber/init.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/task/task.h"
#include "cyber/tools/cyber_monitor/cyber_topology_message.h"
#include "cyber/tools/cyber_monitor/general_channel_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

#include <chrono>
#include <thread>

#include <atomic>

namespace {
std::atomic<bool> running(true);
void SigResizeHandle(int) { Screen::Instance()->Resize(); }
void SigCtrlHandle(int) { Screen::Instance()->Stop(); running = false; }

void printHelp(const char *cmd_name) {
  std::cout << "Usage:\n"
            << cmd_name << "  [option]\nOption:\n"
            << "   -h print help info\n"
            << "   -c specify one channel\n"
            << "   -hz show topic frequency only\n"
            << "Interactive Command:\n"
            << Screen::InteractiveCmdStr << std::endl;
}

enum COMMAND {
  TOO_MANY_PARAMETER,
  HELP,
  NO_OPTION,
  CHANNEL
};

COMMAND ParseOption(int argc, char *const argv[], std::string *command_val,
                    bool *freq_only) {
  *freq_only = false;
  bool has_channel = false;
  if (argc > 5) {
    return TOO_MANY_PARAMETER;
  }
  int index = 1;
  while (index < argc) {
    const char *opt = argv[index];
    if (opt == nullptr) {
      break;
    }
    if (strcmp(opt, "-h") == 0) {
      return HELP;
    }
    if (strcmp(opt, "-f") == 0 || strcmp(opt, "-hz") == 0) {
      *freq_only = true;
    } else if (strcmp(opt, "-c") == 0) {
      if (argv[index + 1]) {
        *command_val = argv[index + 1];
        has_channel = true;
        ++index;  // skip value
      }
    }

    ++index;
  }

  if (has_channel) {
    return CHANNEL;
  }
  return NO_OPTION;
}

}  // namespace

int main(int argc, char *argv[]) {
  std::string val;
  bool freq_only = false;

  COMMAND com = ParseOption(argc, argv, &val, &freq_only);

  switch (com) {
    case TOO_MANY_PARAMETER:
      std::cout << "Too many paramtes\n";
    case HELP:
      printHelp(argv[0]);
      return 0;
    default: {}
  }

  apollo::cyber::Init(argv[0]);
  FLAGS_minloglevel = 3;
  FLAGS_alsologtostderr = 0;
  FLAGS_colorlogtostderr = 0;

  CyberTopologyMessage topology_msg(val);

  auto topology_callback =
      [&topology_msg](const apollo::cyber::proto::ChangeMsg &change_msg) {
        apollo::cyber::Async([&topology_msg, change_msg] {
          topology_msg.TopologyChanged(change_msg);
        });
      };

  auto channel_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  if (channel_manager == nullptr) {
    AERROR << "Cyber Service Discovery is not ready.";
    return -1;
  }
  channel_manager->AddChangeListener(topology_callback);

  std::vector<apollo::cyber::proto::RoleAttributes> role_vec;
  channel_manager->GetWriters(&role_vec);
  for (auto &role : role_vec) {
    topology_msg.AddReaderWriter(role, true);
  }

  role_vec.clear();
  channel_manager->GetReaders(&role_vec);
  for (auto &role : role_vec) {
    topology_msg.AddReaderWriter(role, false);
  }

  Screen *s = Screen::Instance();

  signal(SIGWINCH, SigResizeHandle);
  signal(SIGINT, SigCtrlHandle);
  signal(SIGTSTP, SigCtrlHandle);

  if (freq_only) {
    while (running) {
      const auto &channels = topology_msg.Channels();
      for (const auto &item : channels) {
        if (!GeneralChannelMessage::IsErrorCode(item.second)) {
          std::cout << item.first << ": "
                    << item.second->frame_ratio() << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    s->SetCurrentRenderMessage(&topology_msg);

    s->Init();
    s->Run();
  }

  return 0;
}
