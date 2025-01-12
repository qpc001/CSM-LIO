/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CSMLIO_IO_FILE_WRITER_H_
#define CSMLIO_IO_FILE_WRITER_H_

#include <fstream>
#include <functional>
#include <memory>

#include "csmlio/common/port.h"

namespace csmlio {
namespace io {

// Simple abstraction for a file.
class FileWriter {
 public:
  FileWriter() {}
  FileWriter(const FileWriter&) = delete;
  FileWriter& operator=(const FileWriter&) = delete;

  virtual ~FileWriter() {}

  // Write 'data' to the beginning of the file. This is required to overwrite
  // fixed size headers which contain the number of points once we actually know
  // how many points there are.
  virtual bool WriteHeader(const char* data, size_t len) = 0;

  virtual bool Write(const char* data, size_t len) = 0;
  virtual bool Close() = 0;
  virtual std::string GetFilename() = 0;
};

// An Implementation of file using std::ofstream.
class StreamFileWriter : public FileWriter {
 public:
  ~StreamFileWriter() override;

  StreamFileWriter(const std::string& filename);

  bool Write(const char* data, size_t len) override;
  bool WriteHeader(const char* data, size_t len) override;
  bool Close() override;
  std::string GetFilename() override;

 private:
  const std::string filename_;
  std::ofstream out_;
};

using FileWriterFactory =
    std::function<std::unique_ptr<FileWriter>(const std::string& filename)>;

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_FILE_WRITER_H_
