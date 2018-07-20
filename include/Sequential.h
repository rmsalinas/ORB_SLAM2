
#ifndef SEQUENTIAL_H
#define SEQUENTIAL_H
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace ORB_SLAM2
{

class Sequential{
  public:




    template <typename T>
    class Queue
    {
     public:

      T pop()
      {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
          cond_.wait(mlock);
        }
        auto item = queue_.front();
        queue_.pop();
        return item;
      }

      void clear()
      {
        std::unique_lock<std::mutex> mlock(mutex_);
         while (!queue_.empty())
             queue_.pop();
      }

      void push(const T& item)
      {
        std::unique_lock<std::mutex> mlock(mutex_);
        if (queue_.size()==0)
            queue_.push(item);
        mlock.unlock();
        cond_.notify_one();
      }


     private:
      std::queue<T> queue_;
      std::mutex mutex_;
      std::condition_variable cond_;
    };

    static Queue<int> _LocalOptQ,_LoopClosingQ;

    static void localMappingClear();

    static void waitForEndLocalMapping();
    static void endLocalMapping();
    static void endLoopClosing();
    static void waitForEndLoopClosing();
    static void loopClosingClear();

    static bool _playNFFlag;

    static bool playNextFrame(){return _playNFFlag;}
};

};
#endif
