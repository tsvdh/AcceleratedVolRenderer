#pragma once

#include <set>

int main(int argc, char* argv[]) {
    struct Bla {
        int a;
        // bool operator<(const Bla& other) const {
        //     return a < other.a;
        // }
    };

    friend bool operator<(const Bla& a, const Bla& b) {
        return a.a < b.a;
    }

    std::set<Bla> set;
    set.insert(Bla{1});
}
