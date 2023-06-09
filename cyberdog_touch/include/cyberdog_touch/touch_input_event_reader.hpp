// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_TOUCH__TOUCH_INPUT_EVENT_READER_HPP_
#define CYBERDOG_TOUCH__TOUCH_INPUT_EVENT_READER_HPP_

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <linux/input.h>

namespace cyberdog
{
namespace device
{

class InputEventCircularReader
{
public:
  explicit InputEventCircularReader(size_t numEvents);
  ~InputEventCircularReader();
  ssize_t fill(int fd);
  ssize_t readEvent(input_event const ** events);
  void next();

private:
  struct input_event * const mBuffer;
  struct input_event * const mBufferEnd;
  struct input_event * mHead;
  struct input_event * mCurr;
  ssize_t mFreeSpace;
};

}  //  namespace device
}  //  namespace cyberdog

#endif  // CYBERDOG_TOUCH__TOUCH_INPUT_EVENT_READER_HPP_
