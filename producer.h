#ifndef SHM_SERVER_H
#define SHM_SERVER_H
#include <unordered_map>

struct SHMHeader {
    size_t length;
    int futexp;
    size_t start;
    size_t end;
};

/*
class ShmQueue {
public:
    ShmQueue(int fd, int can_consume_fd, int can_produce_fd, size_t size);
    //void Push(const char* memory, size_t length);
private:
    size_t getOneSlot();
private:
    SHMHeader *header_;
    int fd_;
    int can_consume_fd_;
    int can_produce_fd_;
    unordered_set<size_t> sentRequest;
    size_t size_;
};
*/

class ShmServer {
public:
    ShmServer(const std::string &sock_path, size_t size);
    void Accept(struct timespec *ts=NULL);
    int fd() {return sockfd_;}
    void Start();
private:
    int sockfd_;
    int can_consume_fd_;
    int can_produce_fd_;
    int size_;
};


#endif
