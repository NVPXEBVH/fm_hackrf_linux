#pragma once
#include <vector>
#include <mutex>
#include <condition_variable>

class SharedAudioBuffer {
public:
    SharedAudioBuffer(size_t size);
    ~SharedAudioBuffer() = default;

    void write(const void* data, size_t bytes);
    void read(void* data, size_t bytes);
    int Size();
    bool empty() const;
    void stop();
    void clear();

private:
    std::vector<char> buffer_;
    size_t bufferSize_;
    size_t readPos_ = 0;
    size_t writePos_ = 0;
    bool full_ = false;

    std::mutex mtx_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    bool stopped_ = false;
};