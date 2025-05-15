#include "buffer.h"
#include <cstring>
#include <stdexcept>
#include <mutex>

SharedAudioBuffer::SharedAudioBuffer(size_t size)
    : buffer_(size), bufferSize_(size) {}

void SharedAudioBuffer::write(const void* data, size_t bytes) {
    std::unique_lock<std::mutex> lock(mtx_);

    not_full_.wait(lock, [this]{ return !full_ || stopped_; });
    if (stopped_) return;

    const char* src = static_cast<const char*>(data);
    size_t space = bufferSize_ - writePos_;
    size_t to_write = std::min(bytes, space);

    memcpy(buffer_.data() + writePos_, src, to_write);
    writePos_ += to_write;
    if (writePos_ == bufferSize_) writePos_ = 0;
    full_ = (writePos_ == readPos_);

    not_empty_.notify_one();
}
void SharedAudioBuffer::clear() {
    std::lock_guard<std::mutex> lock(mtx_);

    // Сбрасываем указатели и флаг полноты
    readPos_ = 0;
    writePos_ = 0;
    full_ = false;

    // Уведомляем все ожидающие потоки
    not_empty_.notify_all();
    not_full_.notify_all();
}
void SharedAudioBuffer::read(void* data, size_t bytes) {
    std::unique_lock<std::mutex> lock(mtx_);

    not_empty_.wait(lock, [this]{ return !empty() || stopped_; });
    if (stopped_ && empty()) return;

    char* dst = static_cast<char*>(data);
    size_t available = (writePos_ >= readPos_) ? (writePos_ - readPos_) : (bufferSize_ - readPos_);
    size_t to_read = std::min(bytes, available);

    memcpy(dst, buffer_.data() + readPos_, to_read);
    readPos_ += to_read;
    if (readPos_ == bufferSize_) readPos_ = 0;
    full_ = false;

    not_full_.notify_one();
}
int SharedAudioBuffer::Size()
{
std::lock_guard<std::mutex> lock(mtx_);
    if (full_) return bufferSize_;
    if (writePos_ >= readPos_) {
        return writePos_ - readPos_;
    } else {
        return bufferSize_ - readPos_ + writePos_;
    }
}
bool SharedAudioBuffer::empty() const {
    return (readPos_ == writePos_ && !full_);
}

void SharedAudioBuffer::stop() {
    std::unique_lock<std::mutex> lock(mtx_);
    stopped_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
}