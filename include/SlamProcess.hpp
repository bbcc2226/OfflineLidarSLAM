#include <memory>
#include <DataType.hpp>

class SlamProcess{
public:
    SlamProcess();
    ~SlamProcess();
    SlamProcess(const SlamProcess& other)= delete;
    SlamProcess& operator=(const SlamProcess& other) = delete;
    void Join();
    bool IsFinished() const; 
private:
    struct Impl;
    std::unique_ptr<Impl> process_impl_;
     
};