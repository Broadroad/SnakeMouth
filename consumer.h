#ifndef SHM_CLIENT_H
#define SHM_CLIENT_H
#include <string>

class ShmClient {
public:
    ShmClient(const std::string &sock_path, size_t size);
    void Connect();
private:
    int sockfd_;
    int can_consume_fd_;
    int can_produce_fd_;
    int size_;
};

#endif
