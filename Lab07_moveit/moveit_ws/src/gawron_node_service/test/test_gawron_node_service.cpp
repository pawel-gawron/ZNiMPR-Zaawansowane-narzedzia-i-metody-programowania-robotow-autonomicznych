// Copyright 2023 Pawel_Gawron
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "gawron_node_service/gawron_node_service.hpp"

TEST(TestGawronNodeService, TestHello) {
  std::unique_ptr<gawron_node_service::GawronNodeService> gawron_node_service_ =
    std::make_unique<gawron_node_service::GawronNodeService>();
  auto result = gawron_node_service_->foo(999);
  EXPECT_EQ(result, 999);
}
