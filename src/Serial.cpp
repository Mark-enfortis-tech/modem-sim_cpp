// In Serial.cpp, update the MessageQueue::push method:
bool MessageQueue::push(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(queueMutex);
    
    if (messages.size() >= maxQueueSize) {
        return false;
    }
    
    messages.push(QueueMessage(data));  // Changed from Message to QueueMessage
    return true;
}
