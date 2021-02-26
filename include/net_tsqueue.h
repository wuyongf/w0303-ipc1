#pragma once

#include "../include/net_common.h"

namespace yf
{
    namespace net
    {
        template<typename T>
        class tsqueue
        {
        public:
            tsqueue() = default;
            // delete copy!
            tsqueue(const tsqueue<T>&) = delete;
            ~tsqueue() { clear(); }

        public:
            // returns the size of Queue


            // returns and maintains item at front of Queue
            const T& front()
            {
                std::scoped_lock lock(muxQueue);
                return deQueue.front();
            }

            // returns and maintains item at back of Queue
            const T& back()
            {
                std::scoped_lock lock(muxQueue);
                return deQueue.back();
            }

            // add an item to back of Queue
            void push_back(const T& item)
            {
                std::scoped_lock lock(muxQueue);
                deQueue.emplace_back(std::move(item));

                std::unique_lock<std::mutex> ul(muxBlocking);
                cvBlocking.notify_one();
            }

            // add an item to front of Queue
            void push_front(const T& item)
            {
                std::scoped_lock lock(muxQueue);
                deQueue.emplace_front(std::move(item));

                std::unique_lock<std::mutex> ul(muxBlocking);
                cvBlocking.notify_one();
            }

            // return true if Queue has no items
            bool is_empty()
            {
                std::scoped_lock lock(muxQueue);
                return deQueue.empty();
            }

            // returns number of items in Queue
            size_t count()
            {
                std::scoped_lock lock(muxQueue);
                return deQueue.size();
            }

            // clear Queue
            void clear()
            {
                std::scoped_lock lock(muxQueue);
                return deQueue.clear();
            }

            // removes and returns the item from front of Queue
            T pop_front()
            {
                std::scoped_lock lock(muxQueue);
                auto t = std::move(deQueue.front());
                deQueue.pop_front();
                return t;
            }

            // removes and returns the item from back of Queue
            T pop_back()
            {
                std::scoped_lock lock(muxQueue);
                auto t = std::move(deQueue.back());
                deQueue.pop_back();
                return t;
            }

            // wait for condition variable
            void wait()
            {
                while(is_empty())
                {
                    std::unique_lock<std::mutex> ul(muxBlocking);        // create a unique lock, not blocking now.
                    cvBlocking.wait(ul);                                  // blocking now! wait for "ul" to be notified.
                }
            }
        protected:
            std::mutex muxQueue;
            std::deque<T> deQueue;

            std::mutex muxBlocking;
            std::condition_variable cvBlocking;
        };
    }
}

