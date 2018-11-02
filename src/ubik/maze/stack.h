#pragma once

#include <cstddef>
#include <cassert>

// interface, allows polimorphic usage of either static or dynamic stack
template<typename T>
class Stack {
public:
    virtual void push(T elem) = 0;
    virtual T pop() = 0;
    virtual T top() = 0;
    virtual size_t size() = 0;
    virtual bool is_empty() = 0;
    virtual void empty() = 0;
};


/*
 * Stack allocated using static buffer by specifying maximum stack capacity.
 * It is optimized for small types (takes and returns by copy),
 * for larger types, pointers should be used (e.g. StaticStack<BigObject*, 100>).
 *
 * On reaching the limit size it will panic!
 * pop() from empty stack will panic!
 */
template<typename T, size_t size_limit>
class StaticStack: public Stack<T> {
    int itop;
    T elements[size_limit];
public:
    StaticStack() {
        empty();
    }
    virtual void push(T elem) override {
        itop++;
        assert(itop < size_limit);
        elements[itop] = elem;
    }
    virtual T pop() override {
        assert(itop >= 0);
        return elements[itop--];
    }
    virtual T top() override {
        assert(itop >= 0);
        return elements[itop];
    }
    virtual size_t size() override {
        return itop + 1;
    }
    virtual bool is_empty() override {
        return size() <= 0;
    }
    virtual void empty() override {
        itop = -1;
    }
};
